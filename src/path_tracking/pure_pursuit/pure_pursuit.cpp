#include "path_tracking/pure_pursuit/pure_pursuit.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace path_tracking {

PurePursuit::PurePursuit(const PurePursuitConfig& config)
    : config_(config) {}

void PurePursuit::SetPath(const std::vector<Point2D>& waypoints) {
    path_ = waypoints;
    current_path_idx_ = 0;
    seeded_           = false;
    last_lookahead_   = {};
}

float PurePursuit::ComputeSteering(const vehicle::VehicleState& state) const {
    if (path_.empty()) return 0.0f;

    float ld = config_.lookahead_distance + config_.lookahead_gain * std::abs(state.speed);

    int closest = FindClosestWaypoint(state);
    last_lookahead_ = FindLookaheadPoint(state, closest, ld);

    float dx = last_lookahead_.x - state.x;
    float dy = last_lookahead_.y - state.y;
    float local_y = -dx * std::sin(state.heading) + dy * std::cos(state.heading);

    if (ld * ld < 1e-6f) return 0.0f;

    // δ = atan2(2·L·sin(α), Ld) — from Pure Pursuit derivation
    return std::atan2(2.0f * config_.wheelbase * local_y, ld * ld);
}

int PurePursuit::FindClosestWaypoint(const vehicle::VehicleState& state) const {
    // Forward-only search: scan from current index within a window to prevent backtracking
    // First call after SetPath(): search the whole path, so a rebuild mid-run
    // (live parameter change) picks up where the vehicle really is.
    int start = seeded_ ? current_path_idx_ : 0;
    int end   = seeded_ ? std::min((int)path_.size(), start + config_.search_window) : (int)path_.size();
    seeded_   = true;
    int idx   = start;
    float min_dist = std::numeric_limits<float>::max();
    for (int i = start; i < end; ++i) {
        float dx = path_[i].x - state.x;
        float dy = path_[i].y - state.y;
        float d  = dx * dx + dy * dy;
        if (d < min_dist) { min_dist = d; idx = i; }
    }
    current_path_idx_ = idx;  // advance — never decreases
    return idx;
}

// Point on the path at exactly distance `ld` from the rear axle, interpolated
// on the segment where the distance first reaches ld. The walk is bounded by
// arc length (ld + 10 m past the closest waypoint), not by the path's end.
Point2D PurePursuit::FindLookaheadPoint(const vehicle::VehicleState& state, int closest_idx, float ld) const {
    const int n = static_cast<int>(path_.size());
    auto dist = [&](const Point2D& p) { return std::hypot(p.x - state.x, p.y - state.y); };
    float walked = 0.0f;
    for (int i = closest_idx; i + 1 < n; ++i) {
        const Point2D& a = path_[i];
        const Point2D& b = path_[i + 1];
        if (dist(b) >= ld) {
            // solve |a + t (b - a) - p| = ld for t in [0, 1]
            const float ex = b.x - a.x, ey = b.y - a.y;
            const float fx = a.x - state.x, fy = a.y - state.y;
            const float A = ex * ex + ey * ey;
            const float B = 2.0f * (fx * ex + fy * ey);
            const float C = fx * fx + fy * fy - ld * ld;
            const float disc = B * B - 4.0f * A * C;
            float t = 1.0f;
            if (A > 1e-9f && disc >= 0.0f) t = std::max(0.0f, std::min(1.0f, (-B + std::sqrt(disc)) / (2.0f * A)));
            return {a.x + t * ex, a.y + t * ey};
        }
        walked += std::hypot(b.x - a.x, b.y - a.y);
        if (walked > ld + 10.0f) return b;
    }
    return path_.back();
}

} // namespace path_tracking
