#include "vehicle/vehicle_bicycle_model.hpp"
#include <cmath>
#include <algorithm>

namespace vehicle {

// ============================================================
// KinematicBicycleModel
// ============================================================

KinematicBicycleModel::KinematicBicycleModel(const VehicleData& data)
    : data_(data) {}

VehicleState KinematicBicycleModel::Step(const VehicleState& s,
                                          const VehicleControls& u,
                                          float dt) const
{
    const float L  = data_.wheelbase;
    const float b  = data_.b;
    const float CA = data_.CA;

    // Max front-wheel angle from steering wheel ratio: δ_max = (max_SW_angle / ratio) * π/180
    const float max_delta = (data_.max_steering_wheel_angle / data_.steering_wheel_to_tire_angle_ratio)
                            * (3.14159265f / 180.0f);
    float delta = std::max(-max_delta, std::min(max_delta, u.steering_angle));

    float vx = s.speed;

    // Rear-axle-referenced kinematics (standard Pure-Pursuit formulation)
    float dx   = vx * std::cos(s.heading);
    float dy   = vx * std::sin(s.heading);
    float dpsi = (std::abs(L) > 1e-6f) ? vx / L * std::tan(delta) : 0.0f;
    float dvx  = u.acceleration - CA * vx * std::abs(vx);

    VehicleState next;
    next.x             = s.x       + dx   * dt;
    next.y             = s.y       + dy   * dt;
    next.heading       = s.heading + dpsi * dt;
    next.speed         = std::max(0.0f, vx + dvx * dt);
    next.lateral_speed = 0.0f;
    next.yaw_rate      = dpsi;
    return next;
}

// ============================================================
// DynamicBicycleModel
// ============================================================

DynamicBicycleModel::DynamicBicycleModel(const VehicleData& data)
    : data_(data)
{
    const float m  = data.mass;
    const float a  = data.a;
    const float b  = data.b;
    const float L  = data.wheelbase;
    const float g  = 9.81f;

    // Static axle loads
    float Fz_front = m * g * b / L;
    float Fz_rear  = m * g * a / L;

    // Cornering stiffness: empirically ~6 × normal load [N/rad] for truck/bus tires
    Cf_ = 6.0f * Fz_front;
    Cr_ = 6.0f * Fz_rear;

    // Yaw inertia estimate: Iz ≈ m · a · b  (standard bicycle-model approximation)
    Iz_ = m * a * b;
}

VehicleState DynamicBicycleModel::KinematicFallback(const VehicleState& s,
                                                     const VehicleControls& u,
                                                     float dt) const
{
    KinematicBicycleModel kin(data_);
    return kin.Step(s, u, dt);
}

VehicleState DynamicBicycleModel::Step(const VehicleState& s,
                                        const VehicleControls& u,
                                        float dt) const
{
    if (s.speed < kMinVx)
        return KinematicFallback(s, u, dt);

    const float m  = data_.mass;
    const float a  = data_.a;
    const float b  = data_.b;
    const float CA = data_.CA;

    float vx    = s.speed;
    float vy    = s.lateral_speed;
    float r     = s.yaw_rate;
    float delta = u.steering_angle;

    // Slip angles (linear tire model)
    float alpha_f =  delta - (vy + a * r) / vx;
    float alpha_r =        - (vy - b * r) / vx;

    // Lateral tire forces
    float Fy_f = Cf_ * alpha_f;
    float Fy_r = Cr_ * alpha_r;

    // Aerodynamic drag (mass-normalised: F_drag = m·CA·vx²)
    float ax_drag = -CA * vx * std::abs(vx);

    // Equations of motion (vehicle frame)
    float dvx = u.acceleration + ax_drag + vy * r;
    float dvy = (Fy_f + Fy_r) / m - vx * r;
    float dr  = (a * Fy_f - b * Fy_r) / Iz_;

    // Global frame integration (Euler forward)
    float psi = s.heading;
    VehicleState next;
    next.speed         = std::max(0.0f, vx + dvx * dt);
    next.lateral_speed = vy + dvy * dt;
    next.yaw_rate      = r  + dr  * dt;
    next.heading       = psi + r  * dt;
    next.x = s.x + (vx * std::cos(psi) - vy * std::sin(psi)) * dt;
    next.y = s.y + (vx * std::sin(psi) + vy * std::cos(psi)) * dt;
    return next;
}

} // namespace vehicle
