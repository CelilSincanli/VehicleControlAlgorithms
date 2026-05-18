#include "path_tracking/pure_pursuit/pure_pursuit.hpp"
#include <cmath>
#include <limits>

namespace path_tracking {

PurePursuit::PurePursuit(const PurePursuitConfig& config)
    : config_(config) {}

void PurePursuit::SetPath(const std::vector<Point2D>& waypoints) {
    path_ = waypoints;
}

float PurePursuit::ComputeSteering(const VehicleState& state) const {
    if (path_.empty()) return 0.0f;

    int closest = FindClosestWaypoint(state);
    Point2D target = FindLookaheadPoint(state, closest);

    // Transform target into vehicle-local frame
    float dx = target.x - state.x;
    float dy = target.y - state.y;
    float local_y = -dx * std::sin(state.heading) + dy * std::cos(state.heading);

    float ld2 = dx * dx + dy * dy;
    if (ld2 < 1e-6f) return 0.0f;

    float curvature = 2.0f * local_y / ld2;
    return std::atan(curvature * config_.wheelbase);
}

int PurePursuit::FindClosestWaypoint(const VehicleState& state) const {
    int   idx      = 0;
    float min_dist = std::numeric_limits<float>::max();
    for (int i = 0; i < (int)path_.size(); ++i) {
        float dx = path_[i].x - state.x;
        float dy = path_[i].y - state.y;
        float d  = dx * dx + dy * dy;
        if (d < min_dist) { min_dist = d; idx = i; }
    }
    return idx;
}

Point2D PurePursuit::FindLookaheadPoint(const VehicleState& state, int closest_idx) const {
    float ld = config_.lookahead_distance;
    for (int i = closest_idx; i < (int)path_.size(); ++i) {
        float dx = path_[i].x - state.x;
        float dy = path_[i].y - state.y;
        if (std::sqrt(dx * dx + dy * dy) >= ld) return path_[i];
    }
    return path_.back();
}

} // namespace path_tracking
