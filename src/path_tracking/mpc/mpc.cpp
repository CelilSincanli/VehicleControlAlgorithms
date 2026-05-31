#include "path_tracking/mpc/mpc.hpp"
#include "vehicle/vehicle_state.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace path_tracking {

Mpc::Mpc(const MpcConfig& config) : config_(config) {}

void Mpc::SetPath(const std::vector<Point2D>& waypoints) {
    path_ = waypoints;
    current_path_idx_ = 0;
    target_point_ = path_.empty() ? Point2D{} : path_.front();

    path_yaws_.resize(path_.size());
    const int n = static_cast<int>(path_.size());
    for (int i = 0; i < n; ++i) {
        int prev = std::max(i - 1, 0);
        int next = std::min(i + 1, n - 1);
        path_yaws_[i] = std::atan2(path_[next].y - path_[prev].y,
                                    path_[next].x - path_[prev].x);
    }

    u_.assign(config_.horizon_N, 0.0f);
}

int Mpc::FindNearestWaypoint(float x, float y, int from_idx) const {
    const int n   = static_cast<int>(path_.size());
    const int end = std::min(from_idx + config_.search_window, n);
    float best    = std::numeric_limits<float>::max();
    int   result  = from_idx;
    for (int i = from_idx; i < end; ++i) {
        float dx = path_[i].x - x;
        float dy = path_[i].y - y;
        float d2 = dx * dx + dy * dy;
        if (d2 < best) { best = d2; result = i; }
    }
    return result;
}

float Mpc::RolloutCost(float x0, float y0, float psi0,
                        const std::vector<float>& controls,
                        int start_path_idx) const {
    const float v  = std::max(config_.max_speed * 0.5f, 0.1f);
    const float L  = config_.wheelbase;
    const float dt = config_.rollout_dt;
    const int   N  = static_cast<int>(controls.size());

    float x   = x0;
    float y   = y0;
    float psi = psi0;
    float cost = 0.0f;
    int   idx  = start_path_idx;

    for (int t = 0; t < N; ++t) {
        x   += v * std::cos(psi) * dt;
        y   += v * std::sin(psi) * dt;
        psi += (v / L) * std::tan(controls[t]) * dt;

        idx = FindNearestWaypoint(x, y, idx);
        float pw  = path_yaws_[idx];
        float e_lat = -std::sin(pw) * (x - path_[idx].x)
                     + std::cos(pw) * (y - path_[idx].y);
        float e_psi = psi - pw;
        while (e_psi >  static_cast<float>(M_PI)) e_psi -= 2.0f * static_cast<float>(M_PI);
        while (e_psi < -static_cast<float>(M_PI)) e_psi += 2.0f * static_cast<float>(M_PI);

        cost += config_.w_lat * e_lat * e_lat
              + config_.w_heading * e_psi * e_psi
              + config_.w_control * controls[t] * controls[t];
    }
    return cost;
}

float Mpc::ComputeSteering(const vehicle::VehicleState& state) const {
    if (path_.size() < 2) return 0.0f;

    const int   N  = config_.horizon_N;
    const float fd = config_.fd_step;
    const float lr = config_.learning_rate;

    // Shift warm-start one step (receding horizon)
    std::rotate(u_.begin(), u_.begin() + 1, u_.end());
    u_.back() = 0.0f;

    int nearest = FindNearestWaypoint(state.x, state.y, current_path_idx_);
    current_path_idx_ = nearest;
    target_point_ = path_[nearest];

    // Gradient descent with forward finite differences
    for (int iter = 0; iter < config_.iterations; ++iter) {
        float j0 = RolloutCost(state.x, state.y, state.heading, u_, nearest);

        std::vector<float> grad(N);
        for (int t = 0; t < N; ++t) {
            float saved = u_[t];
            u_[t] = std::max(-config_.max_delta, std::min(config_.max_delta, saved + fd));
            grad[t] = (RolloutCost(state.x, state.y, state.heading, u_, nearest) - j0) / fd;
            u_[t] = saved;
        }

        for (int t = 0; t < N; ++t) {
            u_[t] -= lr * grad[t];
            u_[t] = std::max(-config_.max_delta, std::min(config_.max_delta, u_[t]));
        }
    }

    return u_[0];
}

} // namespace path_tracking
