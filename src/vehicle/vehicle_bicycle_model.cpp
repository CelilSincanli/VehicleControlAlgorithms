#include "vehicle/vehicle_bicycle_model.hpp"
#include "vehicle/vehicle_model_factory.hpp"
#include <cmath>
#include <algorithm>

namespace vehicle {

KinematicBicycleModel::KinematicBicycleModel(const VehicleData& data)
    : data_(data) {}

VehicleState KinematicBicycleModel::Step(const VehicleState& s,
                                          const VehicleControls& u,
                                          float dt)
{
    const float L  = data_.wheelbase;
    const float CA = data_.CA;

    const TireAngleRange lim = TireAngleLimits(data_);
    const float delta = std::max(lim.min, std::min(lim.max, u.steering_angle));

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
    next.steering_angle = delta;

    return next;
}

} // namespace vehicle
