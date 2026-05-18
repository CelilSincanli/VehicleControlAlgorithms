#include "path_tracking/pure_pursuit/pure_pursuit.hpp"
#include <cmath>
#include <limits>

namespace path_tracking {

PurePursuit::PurePursuit(const PurePursuitConfig& config)
    : config_(config) {}

void PurePursuit::SetPath(const std::vector<Point2D>& waypoints) {
    path_ = waypoints;
    current_path_idx_ = 0;
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
    current_path_idx_ = idx;  // advance — never decreases
    return idx;
}

Point2D PurePursuit::FindLookaheadPoint(const vehicle::VehicleState& state, int closest_idx, float ld) const {
    for (int i = closest_idx; i < (int)path_.size(); ++i) {
        float dx = path_[i].x - state.x;
        float dy = path_[i].y - state.y;
        if (std::sqrt(dx * dx + dy * dy) >= ld) return path_[i];
    }
    return path_.back();
}

} // namespace path_tracking
