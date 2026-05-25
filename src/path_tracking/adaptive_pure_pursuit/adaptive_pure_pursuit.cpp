#include "path_tracking/adaptive_pure_pursuit/adaptive_pure_pursuit.hpp"
#include <cmath>
#include <limits>

namespace path_tracking {

AdaptivePurePursuit::AdaptivePurePursuit(const AdaptivePurePursuitConfig& config)
    : config_(config) {}

void AdaptivePurePursuit::SetPath(const std::vector<Point2D>& waypoints) {
    path_ = waypoints;
    current_path_idx_ = 0;
    last_lookahead_   = {};
}

float AdaptivePurePursuit::ComputeSteering(const vehicle::VehicleState& state) const {
    if (path_.empty()) return 0.0f;

    int closest = FindClosestWaypoint(state);

    float kappa  = ComputeCurvature(closest);
    float ld_raw = config_.min_lookahead + config_.speed_gain * std::abs(state.speed);
    float ld     = ld_raw / (1.0f + config_.curvature_gain * kappa);
    ld = std::max(config_.min_lookahead, std::min(config_.max_lookahead, ld));

    last_lookahead_ = FindLookaheadPoint(state, closest, ld);

    float dx      = last_lookahead_.x - state.x;
    float dy      = last_lookahead_.y - state.y;
    float local_y = -dx * std::sin(state.heading) + dy * std::cos(state.heading);

    if (ld * ld < 1e-6f) return 0.0f;

    // δ = atan2(2·L·local_y, Ld²) — Pure Pursuit formula
    return std::atan2(2.0f * config_.wheelbase * local_y, ld * ld);
}

int AdaptivePurePursuit::FindClosestWaypoint(const vehicle::VehicleState& state) const {
    int start = current_path_idx_;
    int end   = std::min((int)path_.size(), start + kSearchWindow);
    int idx   = start;
    float min_dist = std::numeric_limits<float>::max();
    for (int i = start; i < end; ++i) {
        float dx = path_[i].x - state.x;
        float dy = path_[i].y - state.y;
        float d  = dx * dx + dy * dy;
        if (d < min_dist) { min_dist = d; idx = i; }
    }
    current_path_idx_ = idx;
    return idx;
}

Point2D AdaptivePurePursuit::FindLookaheadPoint(const vehicle::VehicleState& state,
                                                  int closest_idx, float ld) const {
    for (int i = closest_idx; i < (int)path_.size(); ++i) {
        float dx = path_[i].x - state.x;
        float dy = path_[i].y - state.y;
        if (std::sqrt(dx * dx + dy * dy) >= ld) return path_[i];
    }
    return path_.back();
}

float AdaptivePurePursuit::ComputeCurvature(int idx) const {
    int i = std::max(0, idx - 1);
    int j = idx;
    int k = std::min((int)path_.size() - 1, idx + 1);
    if (i == j || j == k) return 0.0f;

    float ax = path_[j].x - path_[i].x, ay = path_[j].y - path_[i].y;
    float bx = path_[k].x - path_[i].x, by = path_[k].y - path_[i].y;
    float cross = std::abs(ax * by - ay * bx);
    float la    = std::hypot(ax, ay);
    float lb    = std::hypot(bx - ax, by - ay);
    float lc    = std::hypot(bx, by);
    float denom = la * lb * lc;
    return (denom < 1e-9f) ? 0.0f : 2.0f * cross / denom;
}

} // namespace path_tracking
