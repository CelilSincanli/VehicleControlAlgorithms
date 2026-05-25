#include "path_tracking/stanley/stanley.hpp"
#include "vehicle/vehicle_state.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace path_tracking {

Stanley::Stanley(const StanleyConfig& config) : config_(config) {}

void Stanley::SetPath(const std::vector<Point2D>& waypoints) {
    path_             = waypoints;
    current_path_idx_ = 0;
    target_point_     = path_.empty() ? Point2D{} : path_.front();
}

float Stanley::ComputePathYaw(int index) const {
    const int n = static_cast<int>(path_.size());
    int prev = std::max(index - 1, 0);
    int next = std::min(index + 1, n - 1);
    return std::atan2(path_[next].y - path_[prev].y,
                      path_[next].x - path_[prev].x);
}

int Stanley::FindNearestWaypoint(float fx, float fy) const {
    const int n      = static_cast<int>(path_.size());
    const int end    = std::min(current_path_idx_ + kSearchWindow, n);
    float     best   = std::numeric_limits<float>::max();
    int       result = current_path_idx_;

    for (int i = current_path_idx_; i < end; ++i) {
        float dx = path_[i].x - fx;
        float dy = path_[i].y - fy;
        float d2 = dx * dx + dy * dy;
        if (d2 < best) { best = d2; result = i; }
    }
    return result;
}

float Stanley::ComputeSteering(const vehicle::VehicleState& state) const {
    if (path_.empty()) return 0.0f;

    float fx = state.x + config_.wheelbase * std::cos(state.heading);
    float fy = state.y + config_.wheelbase * std::sin(state.heading);

    int nearest    = FindNearestWaypoint(fx, fy);
    current_path_idx_ = nearest;
    target_point_     = path_[nearest];

    float path_yaw = ComputePathYaw(nearest);

    float heading_error = state.heading - path_yaw;
    while (heading_error >  M_PI) heading_error -= 2.0f * static_cast<float>(M_PI);
    while (heading_error < -M_PI) heading_error += 2.0f * static_cast<float>(M_PI);

    float wp_x = path_[nearest].x;
    float wp_y = path_[nearest].y;
    float e_lat = -std::sin(path_yaw) * (fx - wp_x)
                +  std::cos(path_yaw) * (fy - wp_y);

    float speed_guard = std::max(std::abs(state.speed), config_.min_speed);

    float delta = -(heading_error + std::atan2(config_.stanley_gain * e_lat, speed_guard));

    // Clamp to a sensible steering limit (±1 rad ≈ ±57°, matches bus geometry).
    return std::max(-1.0f, std::min(1.0f, delta));
}

} // namespace path_tracking
