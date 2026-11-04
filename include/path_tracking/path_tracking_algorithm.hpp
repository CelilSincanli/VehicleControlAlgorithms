#pragma once
#include "map/map_data.hpp"
#include "vehicle/vehicle_state.hpp"
#include <vector>

namespace path_tracking {

// Diagnostics from the most recent ComputeSteering(), for algorithms that
// track error against the real current pose and/or run an internal
// optimizer. Populated by Mpc; algorithms with nothing to report (Stanley,
// PurePursuit, LQR, AdaptivePurePursuit, MPPI) just keep the default.
struct TrackingDiagnostics {
    float cross_track_error_m = 0.0f;  // signed lateral error at the real current pose [m]
    float heading_error_deg   = 0.0f;  // state.heading - path tangent at nearest_idx, wrapped
    int   nearest_idx         = -1;
    float last_cost           = 0.0f;  // optimizer-based algorithms only
    bool  steering_at_clamp   = false; // true if ComputeSteering()'s output == +-max_delta
};

class IPathTrackingAlgorithm {
public:
    virtual ~IPathTrackingAlgorithm() = default;
    virtual void    SetPath(const std::vector<Point2D>& waypoints) = 0;
    virtual float   ComputeSteering(const vehicle::VehicleState& state) const = 0;
    virtual Point2D GetLookaheadPoint() const = 0;

    // Steering-rate limit [rad/s] of the actuator the vehicle model applies.
    // Called when the vehicle model changes mid-run, so actuator-aware
    // algorithms (LQR, MPC, MPPI) adapt without losing their state.
    virtual void SetActuatorRate(float max_delta_rate) { (void)max_delta_rate; }

    // Optional: fills `out` with diagnostics from the last ComputeSteering()
    // and returns true, or returns false if this algorithm has nothing to report.
    virtual bool GetLastTrackingDiagnostics(TrackingDiagnostics& out) const { (void)out; return false; }
};

} // namespace path_tracking
