#pragma once
#include <string>

namespace vehicle {

// ---------------------------------------------------------------------------
// Subsystem parameter blocks, one per optional nested JSON object. Each
// carries its own `enabled` flag so VehicleModelFactory can tell "block
// present in JSON / live-tuned in the UI, opt in" apart from "absent, use
// today's default behavior" — the same role VehicleData::loaded plays for
// the whole file. All defaults reproduce today's exact behavior.
// ---------------------------------------------------------------------------

struct TireParams {
    bool  enabled                   = false;
    float cornering_stiffness_coeff = 6.0f;  // multiplier applied to each axle's
                                              // static Fz to derive Cf/Cr — an
                                              // empirical truck/bus tire heuristic
    float friction_coefficient      = 1.0f;  // μ — friction-circle clamp
};

struct SuspensionParams {
    bool enabled = false;  // false => static axle split only, cog_height ignored (today's behavior)
};

struct SteeringParams {
    bool enabled         = false;  // false => InstantaneousSteeringModel (today's behavior)
    bool use_rate_limit  = false;  // true  => RateLimitedSteeringModel, using max_steering_wheel_rate
};

} // namespace vehicle

struct VehicleData {
    std::string name;

    // Dynamics
    float mass                 = 0.0f;  // [kg]
    float a                    = 0.0f;  // distance from front axle to COG [m]
    float b                    = 0.0f;  // distance from rear axle  to COG [m]
    float CA                   = 0.0f;  // air resistance coefficient [1/m]
    float cog_height           = 0.0f;  // [m] — CoG height above ground, for load transfer
    float yaw_inertia          = 0.0f;  // [kg*m^2] — yaw (Z-axis) moment of inertia about the
                                        // CoG. 0.0 = not provided; ComposedVehicleModel falls
                                        // back to the mass*a*b two-point-mass estimate.

    // Longitudinal limits applied by the simulation's speed controller
    float max_acceleration     = 2.0f;  // [m/s^2]
    float max_deceleration     = 4.0f;  // [m/s^2] (positive number)
    float max_jerk             = 2.0f;  // [m/s^3]

    // Steering limits
    float minimum_turning_radius              = 0.0f;  // [m] rear-axle centre radius at full lock; authoritative:
                                                       //     the tire angle is limited to atan(wheelbase / radius)
    float max_steering_wheel_angle            = 0.0f;  // [deg]
    float min_steering_wheel_angle            = 0.0f;  // [deg]
    float max_steering_wheel_rate             = 0.0f;  // [deg/s]
    float steering_wheel_to_tire_angle_ratio  = 1.0f;  // [dimensionless]

    // Geometry
    float wheelbase       = 0.0f;  // [m]
    float wheel_radius    = 0.0f;  // [m]
    float wheel_width     = 0.0f;  // [m]
    float wheel_tread     = 0.0f;  // [m]  (left to right wheel center)
    float front_overhang  = 0.0f;  // [m]
    float rear_overhang   = 0.0f;  // [m]
    float left_overhang   = 0.0f;  // [m]
    float right_overhang  = 0.0f;  // [m]
    float vehicle_height  = 0.0f;  // [m]

    // Subsystem parameter blocks (optional nested JSON objects)
    vehicle::TireParams       tire;
    vehicle::SuspensionParams suspension;
    vehicle::SteeringParams   steering;

    bool loaded = false;
};
