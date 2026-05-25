#include "path_tracking/mppi/mppi.hpp"
#include "vehicle/vehicle_state.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <numeric>

namespace path_tracking {

Mppi::Mppi(const MppiConfig& config) : config_(config) {}

void Mppi::SetPath(const std::vector<Point2D>& waypoints) {
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

    u_.assign(config_.horizon_T, 0.0f);
}

int Mppi::FindNearestWaypoint(float x, float y, int from_idx) const {
    const int n   = static_cast<int>(path_.size());
    const int end = std::min(from_idx + kSearchWindow, n);
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

// Cost of rolling out from (x0,y0,psi0) at constant speed with given steering sequence.
float Mppi::RolloutCost(float x0, float y0, float psi0,
                         const std::vector<float>& controls,
                         int start_path_idx) const {
    const float v  = std::max(config_.max_speed * 0.5f, 0.1f);
    const float L  = config_.wheelbase;
    const float dt = config_.rollout_dt;
    const int   T  = static_cast<int>(controls.size());

    float x   = x0;
    float y   = y0;
    float psi = psi0;
    float cost = 0.0f;
    int   idx  = start_path_idx;

    for (int t = 0; t < T; ++t) {
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
              + config_.w_heading * e_psi * e_psi;
    }
    return cost;
}

float Mppi::ComputeSteering(const vehicle::VehicleState& state) const {
    if (path_.size() < 2) return 0.0f;

    const int T = config_.horizon_T;
    const int K = config_.num_samples_K;

    // Shift warm-start one step (receding horizon)
    std::rotate(u_.begin(), u_.begin() + 1, u_.end());
    u_.back() = 0.0f;

    int nearest = FindNearestWaypoint(state.x, state.y, current_path_idx_);
    current_path_idx_ = nearest;
    target_point_ = path_[nearest];

    // Sample noise and evaluate each trajectory
    std::vector<std::vector<float>> eps(K, std::vector<float>(T));
    std::vector<float> costs(K, 0.0f);

    for (int k = 0; k < K; ++k) {
        std::vector<float> v_seq(T);
        for (int t = 0; t < T; ++t) {
            eps[k][t] = config_.sigma_steer * noise_dist_(rng_);
            float v_kt = u_[t] + eps[k][t];
            v_kt = std::max(-kMaxDelta, std::min(kMaxDelta, v_kt));
            v_seq[t] = v_kt;
        }
        costs[k] = RolloutCost(state.x, state.y, state.heading, v_seq, nearest);
    }

    // Importance weights (numerically stable: shift by min cost)
    float min_cost = *std::min_element(costs.begin(), costs.end());
    std::vector<float> weights(K);
    float weight_sum = 0.0f;
    for (int k = 0; k < K; ++k) {
        weights[k] = std::exp(-(costs[k] - min_cost) / config_.lambda);
        weight_sum += weights[k];
    }

    // Weighted update of control sequence
    for (int t = 0; t < T; ++t) {
        float weighted_eps = 0.0f;
        for (int k = 0; k < K; ++k)
            weighted_eps += weights[k] * eps[k][t];
        u_[t] += weighted_eps / weight_sum;
        u_[t] = std::max(-kMaxDelta, std::min(kMaxDelta, u_[t]));
    }

    return u_[0];
}

} // namespace path_tracking
