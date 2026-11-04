#pragma once
#include "map/map_data.hpp"
#include "vehicle/vehicle_data.hpp"
#include <vector>

namespace path_tracking {

// ---------------------------------------------------------------------------
// Path feasibility for one vehicle
// ---------------------------------------------------------------------------
// Measures a reference path (real metres) against a vehicle's steering and
// tire limits, and derives the highest target speed a rate-limited steering
// actuator can follow:
//   v_lateral = sqrt(kLateralFrictionShare * mu * g / kappa_max)
//   v_steer   = steering_rate_max / max|d(delta)/ds|,  delta(s) = atan(L * kappa(s))
//   speed_cap = min(v_lateral, v_steer), clamped to [kMinSpeedCap, kMaxSpeedCap]
// kappa is the Menger curvature over +-kCurvatureWindow waypoints, the same
// window the LQR / Adaptive Pure Pursuit curvature estimates use.
// ---------------------------------------------------------------------------
struct PathFeasibility {
    float path_min_radius_m    = 0.0f;  // tightest bend of the path
    float vehicle_min_radius_m = 0.0f;  // L / tan(max tire angle), rear axle
    bool  geometrically_feasible = true;
    float steer_needed_rad     = 0.0f;  // tire angle the tightest bend needs
    float v_lateral_mps        = 0.0f;
    float v_steer_rate_mps     = 0.0f;  // +inf when steering is not rate limited
    float speed_cap_mps        = 0.0f;
};

PathFeasibility AnalyzePathFeasibility(const std::vector<Point2D>& path_m, const VehicleData& vd);

} // namespace path_tracking
