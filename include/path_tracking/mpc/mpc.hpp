#pragma once
#include "path_tracking/path_tracking_algorithm.hpp"
#include <limits>
#include <vector>

namespace path_tracking {

struct MpcConfig {
    int   horizon_N     = 15;    // prediction horizon (steps)
    int   iterations    = 30;    // QP solver (FISTA) iterations per call
    float rollout_dt    = 0.1f;  // prediction step [s]
    float control_dt    = 1.0f / 60.0f;  // control period of the simulation loop [s]
    float wheelbase     = 4.0f;  // L [m] — set from VehicleData
    float max_speed     = 5.0f;
    float w_lat         = 1.0f;  // lateral error weight
    float w_heading     = 0.5f;  // heading error weight
    float w_control     = 0.05f; // steering deviation from the path's feed-forward angle
    float w_rate        = 0.1f;  // steering rate [rad/s] weight
    int   search_window = 40;    // max waypoints to scan ahead for nearest point
    float max_delta     = 1.0f;  // steering output clamp [rad]
    float max_delta_rate = std::numeric_limits<float>::max();  // actuator rate limit [rad/s] — set from VehicleData
};

// ---------------------------------------------------------------------------
// Linear MPC (in-house QP)
// ---------------------------------------------------------------------------
// Error dynamics along the path, linearised around the feed-forward angle
// delta_ff = atan(L kappa(s)) of the previewed curvature:
//   e_psi[k+1] = e_psi[k] + dt * v/L * sec^2(delta_ff_k) * (d_k - delta_ff_k)
//   e_lat[k+1] = e_lat[k] + dt * v * e_psi[k]
//   d_k        = delta_applied + sum_{j<=k} Delta_j          (actuator state)
// Decision variables: steering increments Delta_k, |Delta_k| <= rate * dt and
// |d_k| <= max_delta. Cost: w_lat e_lat^2 + w_heading e_psi^2
// + w_control (d - delta_ff)^2 + w_rate (Delta/dt)^2. The QP is solved by
// FISTA with step 1/L from a Gershgorin bound, so it converges monotonically
// and deterministically (the old finite-difference gradient descent did not).
// The output is the applied angle advanced by one control period at the
// planned rate, so it is feasible for the rate-limited actuator.
// ---------------------------------------------------------------------------
class Mpc : public IPathTrackingAlgorithm {
public:
    explicit Mpc(const MpcConfig& config = {});

    void    SetPath(const std::vector<Point2D>& waypoints) override;
    float   ComputeSteering(const vehicle::VehicleState& state) const override;
    Point2D GetLookaheadPoint() const override { return target_point_; }
    void    SetActuatorRate(float max_delta_rate) override { config_.max_delta_rate = max_delta_rate; }

    bool GetLastTrackingDiagnostics(TrackingDiagnostics& out) const override;

    const MpcConfig& GetConfig() const { return config_; }

private:
    MpcConfig            config_;
    std::vector<Point2D> path_;
    std::vector<float>   path_s_;      // arc length at each waypoint [m]
    std::vector<float>   path_kappa_;  // signed curvature at each waypoint [1/m]

    mutable std::vector<float> delta_inc_;   // warm-started solution Delta_k
    mutable Point2D      target_point_{};
    mutable int          current_path_idx_ = 0;
    mutable bool         seeded_ = false;
    mutable float        last_e_lat_ = 0.0f;
    mutable float        last_e_psi_ = 0.0f;
    mutable float        last_cost_  = 0.0f;
    mutable bool         last_steering_at_clamp_ = false;

    // QP scratch buffers, reused across calls (no allocation per tick)
    mutable std::vector<float> H_, g_, grad_, x_prev_, z_;
    mutable std::vector<float> dff_, gain_, c_psi_, c_lat_, f_psi_, f_lat_;
    mutable std::vector<float> M_psi_, M_lat_, MT_psi_, MT_lat_;

    int   FindNearestWaypoint(float x, float y) const;
    float CurvatureAt(float s) const;
    void  ProjectFeasible(std::vector<float>& inc, float d0, float max_step) const;
};

} // namespace path_tracking
