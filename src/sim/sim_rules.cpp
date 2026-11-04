#include "sim/sim_rules.hpp"
#include <algorithm>
#include <cmath>

namespace sim {

float SpeedCommand(float v_target, float v, float a_prev, const VehicleData& vd, float dt) {
    constexpr float kSpeedGain = 1.0f;  // [1/s]
    float a = kSpeedGain * (v_target - v);
    a = std::max(-vd.max_deceleration, std::min(vd.max_acceleration, a));
    const float max_change = vd.max_jerk * dt;
    return std::max(a_prev - max_change, std::min(a_prev + max_change, a));
}

Quad BodyCorners(const vehicle::VehicleState& s, const VehicleData& vd) {
    const float front = vd.wheelbase + vd.front_overhang;
    const float rear  = vd.rear_overhang;
    const float half  = vd.wheel_tread * 0.5f + std::max(vd.left_overhang, vd.right_overhang);
    const float c = std::cos(s.heading), sn = std::sin(s.heading);
    auto at = [&](float f, float l) { return Point2D{s.x + f * c - l * sn, s.y + f * sn + l * c}; };
    return {at(front, half), at(front, -half), at(-rear, -half), at(-rear, half)};
}

namespace {
// Separating-axis test: oriented quad vs axis-aligned rectangle.
bool QuadHitsRect(const Quad& q, const Obstacle& o) {
    const Point2D r[4] = {{o.x, o.y}, {o.x + o.width, o.y}, {o.x + o.width, o.y + o.height}, {o.x, o.y + o.height}};
    const Point2D axes[4] = {{1, 0}, {0, 1}, {q[1].x - q[0].x, q[1].y - q[0].y}, {q[3].x - q[0].x, q[3].y - q[0].y}};
    for (const auto& ax : axes) {
        float a0 = 1e30f, a1 = -1e30f, b0 = 1e30f, b1 = -1e30f;
        for (const auto& p : q) { float d = p.x * ax.x + p.y * ax.y; a0 = std::min(a0, d); a1 = std::max(a1, d); }
        for (const auto& p : r) { float d = p.x * ax.x + p.y * ax.y; b0 = std::min(b0, d); b1 = std::max(b1, d); }
        if (a1 < b0 || b1 < a0) return false;
    }
    return true;
}
} // anonymous namespace

bool BodyHitsObstacle(const Quad& body, const MapData& map) {
    for (const auto& o : map.obstacles)
        if (QuadHitsRect(body, o)) return true;
    return false;
}

bool BodyOffMap(const Quad& body, const MapData& map) {
    for (const auto& p : body)
        if (p.x < 0.0f || p.y < 0.0f || p.x > map.world_width || p.y > map.world_height) return true;
    return false;
}

const char* OutcomeName(Outcome o) {
    switch (o) {
        case Outcome::Running:   return "running";
        case Outcome::Goal:      return "goal reached";
        case Outcome::Collision: return "collision";
        case Outcome::OffMap:    return "left the map";
        case Outcome::Timeout:   return "timeout";
    }
    return "?";
}

float PathLength(const std::vector<Point2D>& path) {
    float len = 0.0f;
    for (size_t i = 1; i < path.size(); ++i)
        len += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    return len;
}

Outcome CheckTermination(const vehicle::VehicleState& s, const VehicleData& vd, const MapData& map,
                         const std::vector<Point2D>& path, float path_length,
                         float elapsed, float v_target, const TerminationRules& rules) {
    // Goal: within the radius, or past the goal line on the last segment and
    // close to the path (a near-miss no longer drives on forever).
    const float gdx = map.goal.x - s.x, gdy = map.goal.y - s.y;
    if (gdx * gdx + gdy * gdy < rules.goal_radius * rules.goal_radius) return Outcome::Goal;
    if (path.size() >= 2 && gdx * gdx + gdy * gdy < rules.goal_capture * rules.goal_capture) {
        const Point2D& a = path[path.size() - 2];
        const Point2D& b = path.back();
        const float tx = b.x - a.x, ty = b.y - a.y, tl = std::hypot(tx, ty);
        if (tl > 1e-6f) {
            const float along   = ((s.x - b.x) * tx + (s.y - b.y) * ty) / tl;   // > 0: past the goal line
            const float lateral = std::abs((s.y - b.y) * tx - (s.x - b.x) * ty) / tl;
            if (along >= 0.0f && lateral < rules.goal_lateral_tol) return Outcome::Goal;
        }
    }

    const Quad body = BodyCorners(s, vd);
    if (BodyHitsObstacle(body, map)) return Outcome::Collision;
    if (BodyOffMap(body, map))       return Outcome::OffMap;

    const float timeout = rules.timeout_factor * path_length / std::max(v_target, 0.5f) + rules.timeout_slack;
    if (elapsed > timeout) return Outcome::Timeout;
    return Outcome::Running;
}

} // namespace sim
