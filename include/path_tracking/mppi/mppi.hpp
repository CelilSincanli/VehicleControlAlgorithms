#pragma once
#include "path_tracking/path_tracking_algorithm.hpp"
#include <vector>
#include <random>

namespace path_tracking {

struct MppiConfig {
    int   horizon_T     = 15;    // prediction horizon (steps)
    int   num_samples_K = 128;   // trajectory samples
    float lambda        = 1.0f;  // temperature — lower concentrates on best samples
    float sigma_steer   = 0.3f;  // steering noise std dev [rad]
    float rollout_dt    = 0.1f;  // rollout time step [s]
    float wheelbase     = 4.0f;  // L [m] — set from VehicleData
    float max_speed     = 5.0f;
    float w_lat         = 1.0f;  // lateral error weight
    float w_heading     = 0.5f;  // heading error weight
    int   search_window = 40;    // max waypoints to scan ahead for nearest point
    float max_delta     = 1.0f;  // steering output clamp [rad]
    int   rng_seed      = 42;    // random seed for reproducible noise sampling
};

class Mppi : public IPathTrackingAlgorithm {
public:
    explicit Mppi(const MppiConfig& config = {});

    void    SetPath(const std::vector<Point2D>& waypoints) override;
    float   ComputeSteering(const vehicle::VehicleState& state) const override;
    Point2D GetLookaheadPoint() const override { return target_point_; }

    const MppiConfig& GetConfig() const { return config_; }

private:
    MppiConfig           config_;
    std::vector<Point2D> path_;
    std::vector<float>   path_yaws_;

    mutable std::vector<float> u_;
    mutable Point2D      target_point_{};
    mutable int          current_path_idx_ = 0;
    mutable std::mt19937 rng_;
    mutable std::normal_distribution<float> noise_dist_{0.0f, 1.0f};

    float RolloutCost(float x0, float y0, float psi0,
                      const std::vector<float>& controls,
                      int start_path_idx) const;
    int   FindNearestWaypoint(float x, float y, int from_idx) const;
    float PathYawAt(int index) const;
};

} // namespace path_tracking
