#pragma once

namespace vehicle {

// Shared state used by both vehicle models and path-tracking algorithms.
// (x, y) is the REAR-AXLE centre for every vehicle model.
struct VehicleState {
    float x             = 0.0f;  // global position x          [m]
    float y             = 0.0f;  // global position y          [m]
    float heading       = 0.0f;  // yaw angle ψ (0 = east)    [rad]
    float speed         = 0.0f;  // longitudinal speed vx      [m/s]
    float lateral_speed = 0.0f;  // lateral speed vy           [m/s]  (dynamic model)
    float yaw_rate      = 0.0f;  // yaw rate r = ψ̇            [rad/s] (dynamic model)
    float steering_angle = 0.0f; // road-wheel angle actually applied after the actuator's
                                 // travel/rate limits [rad]; written by every vehicle model
};

// Control inputs delivered to the vehicle model each step.
struct VehicleControls {
    float steering_angle = 0.0f;  // front-wheel steering angle δ  [rad]
    float acceleration   = 0.0f;  // longitudinal acceleration      [m/s²]
};

} // namespace vehicle
