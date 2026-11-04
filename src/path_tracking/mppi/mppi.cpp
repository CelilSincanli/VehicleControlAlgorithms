#include "path_tracking/mppi/mppi.hpp"
#include "vehicle/vehicle_state.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <numeric>

namespace path_tracking {

Mppi::Mppi(const MppiConfig& config) : config_(config), rng_(config.rng_seed) {}

void Mppi::SetPath(const std::vector<Point2D>& waypoints) {
    path_ = waypoints;
    current_path_idx_ = 0;
    seeded_           = false;
    shift_accum_      = 0.0f;
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

int Mppi::FindNearestWaypoint(float x, float y, int from_idx, bool whole_path) const {
    const int n   = static_cast<int>(path_.size());
    const int end = whole_path ? n : std::min(from_idx + config_.search_window, n);
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

// Cost of rolling out from the current state at its real speed. Each control
// is reached at the actuator's rate from the previously applied angle.
float Mppi::RolloutCost(const vehicle::VehicleState& s0,
                         const std::vector<float>& controls,
                         int start_path_idx) const {
    const float v  = std::max(s0.speed, 0.5f);
    const float L  = config_.wheelbase;
    const float dt = step_dt_;
    const int   T  = static_cast<int>(controls.size());
    const float max_step = config_.max_delta_rate * dt;

    float x   = s0.x;
    float y   = s0.y;
    float psi = s0.heading;
    float d   = s0.steering_angle;
    float cost = 0.0f;
    int   idx  = start_path_idx;

    for (int t = 0; t < T; ++t) {
        d   += std::max(-max_step, std::min(max_step, controls[t] - d));
        x   += v * std::cos(psi) * dt;
        y   += v * std::sin(psi) * dt;
        psi += (v / L) * std::tan(d) * dt;

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
    if (path_.size() < 2 || u_.empty() || config_.num_samples_K <= 0) return 0.0f;

    const int T = config_.horizon_T;
    const int K = config_.num_samples_K;

    // Shift the warm start by one rollout step only once a rollout step of
    // real time has passed (the controller runs every control_dt << rollout_dt).
    shift_accum_ += config_.control_dt;
    step_dt_ = StepDt(state.speed);
    if (shift_accum_ >= step_dt_) {
        shift_accum_ -= step_dt_;
        std::rotate(u_.begin(), u_.begin() + 1, u_.end());
        u_.back() = u_[u_.size() > 1 ? u_.size() - 2 : 0];
    }

    int nearest = FindNearestWaypoint(state.x, state.y, current_path_idx_, !seeded_);
    seeded_ = true;
    current_path_idx_ = nearest;
    target_point_ = path_[nearest];

    // Sample noise and evaluate each trajectory. Buffers are members so the
    // 60 Hz control loop does not allocate (was K+3 heap allocations per call).
    eps_.resize(static_cast<size_t>(K) * T);
    costs_.assign(K, 0.0f);
    v_seq_.resize(T);

    for (int k = 0; k < K; ++k) {
        float* eps_k = &eps_[static_cast<size_t>(k) * T];
        for (int t = 0; t < T; ++t) {
            eps_k[t] = config_.sigma_steer * noise_dist_(rng_);
            float v_kt = u_[t] + eps_k[t];
            v_kt = std::max(-config_.max_delta, std::min(config_.max_delta, v_kt));
            v_seq_[t] = v_kt;
        }
        costs_[k] = RolloutCost(state, v_seq_, nearest);
    }

    // Importance weights (numerically stable: shift by min cost). lambda is
    // floored: lambda = 0 would give 0/0 = NaN weights.
    const float lambda = std::max(config_.lambda, 1e-3f);
    float min_cost = *std::min_element(costs_.begin(), costs_.end());
    weights_.resize(K);
    float weight_sum = 0.0f;
    for (int k = 0; k < K; ++k) {
        weights_[k] = std::exp(-(costs_[k] - min_cost) / lambda);
        weight_sum += weights_[k];
    }

    // Weighted update of control sequence
    for (int t = 0; t < T; ++t) {
        float weighted_eps = 0.0f;
        for (int k = 0; k < K; ++k)
            weighted_eps += weights_[k] * eps_[static_cast<size_t>(k) * T + t];
        u_[t] += weighted_eps / weight_sum;
        u_[t] = std::max(-config_.max_delta, std::min(config_.max_delta, u_[t]));
    }

    // Command for this control period: head for u_[0] at most at the rate the
    // rollout assumed, scaled to one control period (feasible for the actuator).
    const float d0   = state.steering_angle;
    const float step = std::max(-config_.max_delta_rate * step_dt_,
                                std::min(config_.max_delta_rate * step_dt_, u_[0] - d0));
    const float frac = std::min(1.0f, config_.control_dt / step_dt_);
    return std::max(-config_.max_delta, std::min(config_.max_delta, d0 + step * frac));
}

float Mppi::StepDt(float speed) const {
    const float horizon_s = static_cast<float>(std::max(1, config_.horizon_T)) * config_.rollout_dt;
    const float preview   = std::max(speed, 0.5f) * horizon_s;
    const float wanted    = kMinPreviewWheelbases * config_.wheelbase;
    return (preview >= wanted) ? config_.rollout_dt : config_.rollout_dt * wanted / preview;
}

} // namespace path_tracking
