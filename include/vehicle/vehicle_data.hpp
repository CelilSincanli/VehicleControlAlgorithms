#pragma once
#include <string>

struct VehicleData {
    std::string name;

    // Dynamics
    float mass                 = 0.0f;  // [kg]
    float a                    = 0.0f;  // distance from front axle to COG [m]
    float b                    = 0.0f;  // distance from rear axle  to COG [m]
    float CA                   = 0.0f;  // air resistance coefficient [1/m]

    // Steering limits
    float minimum_turning_radius   = 0.0f;  // [m]
    float max_steering_wheel_angle = 0.0f;  // [deg]
    float min_steering_wheel_angle = 0.0f;  // [deg]
    float max_steering_wheel_rate  = 0.0f;  // [deg/s]

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

    bool loaded = false;
};
