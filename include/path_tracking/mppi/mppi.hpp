#pragma once
#include "path_tracking/path_tracking_algorithm.hpp"
#include <vector>
#include <random>

namespace vehicle { struct VehicleState; }

namespace path_tracking {

struct MppiConfig {
    int   horizon_T     = 20;    // prediction horizon (steps)
    int   num_samples_K = 128;   // trajectory samples
    float lambda        = 1.0f;  // temperature — lower concentrates on best samples
    float sigma_steer   = 0.2f;  // steering noise std dev [rad]
    float rollout_dt    = 0.15f; // rollout time step [s] (minimum; stretched at low speed)
    float wheelbase     = 4.0f;  // L [m] — set from VehicleData
    float max_speed     = 5.0f;
    float w_lat         = 1.0f;  // lateral error weight
    float w_heading     = 0.5f;  // heading error weight
    int   search_window = 40;    // max waypoints to scan ahead for nearest point
    float max_delta     = 1.0f;  // steering output clamp [rad]
    int   rng_seed      = 42;    // random seed for reproducible noise sampling
    float max_delta_rate = 1e9f; // actuator rate limit [rad/s] — set from VehicleData
    float control_dt    = 1.0f / 60.0f;  // control period of the simulation loop [s]
};

class Mppi : public IPathTrackingAlgorithm {
public:
    explicit Mppi(const MppiConfig& config = {});

    void    SetPath(const std::vector<Point2D>& waypoints) override;
    float   ComputeSteering(const vehicle::VehicleState& state) const override;
    Point2D GetLookaheadPoint() const override { return target_point_; }
    void    SetActuatorRate(float max_delta_rate) override { config_.max_delta_rate = max_delta_rate; }

    const MppiConfig& GetConfig() const { return config_; }

private:
    MppiConfig           config_;
    std::vector<Point2D> path_;
    std::vector<float>   path_yaws_;

    mutable std::vector<float> u_;
    mutable Point2D      target_point_{};
    mutable int          current_path_idx_ = 0;
    mutable bool         seeded_ = false;      // first call searches the whole path
    mutable float        shift_accum_ = 0.0f;  // time since the warm start was last shifted [s]
    mutable float        step_dt_ = 0.1f;      // rollout step used this call [s] (see StepDt)
    mutable std::mt19937 rng_;
    mutable std::normal_distribution<float> noise_dist_{0.0f, 1.0f};

    // Per-call scratch buffers, reused across calls (no allocation per tick)
    mutable std::vector<float> eps_;      // K x T noise samples, row-major
    mutable std::vector<float> costs_;
    mutable std::vector<float> weights_;
    mutable std::vector<float> v_seq_;

    // Rollout of the kinematic bicycle at the real speed, with the steering
    // rate-limited from the applied angle exactly like the actuator.
    float RolloutCost(const vehicle::VehicleState& s0,
                      const std::vector<float>& controls,
                      int start_path_idx) const;
    int   FindNearestWaypoint(float x, float y, int from_idx, bool whole_path = false) const;
    float PathYawAt(int index) const;
    // Rollout step: rollout_dt, stretched at low speed so the horizon always
    // previews at least kMinPreviewWheelbases wheelbases of path.
    float StepDt(float speed) const;
    static constexpr float kMinPreviewWheelbases = 2.5f;
};

} // namespace path_tracking
