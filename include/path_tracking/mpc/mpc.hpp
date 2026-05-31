#pragma once
#include "path_tracking/path_tracking_algorithm.hpp"
#include <vector>

namespace path_tracking {

struct MpcConfig {
    int   horizon_N     = 15;    // prediction horizon (steps)
    int   iterations    = 10;    // gradient descent iterations per call
    float learning_rate = 0.08f;
    float fd_step       = 0.01f; // finite difference step [rad]
    float rollout_dt    = 0.1f;  // [s]
    float wheelbase     = 4.0f;  // L [m] — set from VehicleData
    float max_speed     = 5.0f;
    float w_lat         = 1.0f;  // lateral error weight
    float w_heading     = 0.5f;  // heading error weight
    float w_control     = 0.05f; // control effort penalty (Σ δ²)
    int   search_window = 40;    // max waypoints to scan ahead for nearest point
    float max_delta     = 1.0f;  // steering output clamp [rad]
};

class Mpc : public IPathTrackingAlgorithm {
public:
    explicit Mpc(const MpcConfig& config = {});

    void    SetPath(const std::vector<Point2D>& waypoints) override;
    float   ComputeSteering(const vehicle::VehicleState& state) const override;
    Point2D GetLookaheadPoint() const override { return target_point_; }

    const MpcConfig& GetConfig() const { return config_; }

private:
    MpcConfig            config_;
    std::vector<Point2D> path_;
    std::vector<float>   path_yaws_;

    mutable std::vector<float> u_;
    mutable Point2D      target_point_{};
    mutable int          current_path_idx_ = 0;

    float RolloutCost(float x0, float y0, float psi0,
                      const std::vector<float>& controls,
                      int start_path_idx) const;
    int   FindNearestWaypoint(float x, float y, int from_idx) const;
};

} // namespace path_tracking
