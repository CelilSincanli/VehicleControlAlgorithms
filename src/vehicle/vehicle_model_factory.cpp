#include "vehicle/vehicle_model_factory.hpp"
#include "vehicle/vehicle_bicycle_model.hpp"
#include "vehicle/composed_vehicle_model.hpp"
#include "vehicle/tire/linear_tire_model.hpp"
#include "vehicle/suspension/rigid_suspension_model.hpp"
#include "vehicle/drivetrain/direct_accel_drivetrain_model.hpp"
#include "vehicle/aero/simple_aero_model.hpp"
#include "vehicle/steering/instantaneous_steering_model.hpp"
#include "vehicle/steering/rate_limited_steering_model.hpp"
#include <algorithm>
#include <cmath>

namespace vehicle {

float ToRadians(float degrees) { return degrees * (3.14159265f / 180.0f); }

TireAngleRange TireAngleLimits(const VehicleData& data) {
    const float ratio = data.steering_wheel_to_tire_angle_ratio;
    TireAngleRange r;
    r.max = ToRadians(data.max_steering_wheel_angle) / ratio;
    r.min = (data.min_steering_wheel_angle < 0.0f) ? ToRadians(data.min_steering_wheel_angle) / ratio : -r.max;
    // minimum_turning_radius is authoritative: the rear axle cannot turn on a
    // tighter circle than it, whatever the steering-wheel travel would allow.
    if (data.minimum_turning_radius > 0.0f && data.wheelbase > 0.0f) {
        const float by_radius = std::atan(data.wheelbase / data.minimum_turning_radius);
        r.max = std::min(r.max, by_radius);
        r.min = std::max(r.min, -by_radius);
    }
    return r;
}

float MaxTireAngle(const VehicleData& data) {
    TireAngleRange r = TireAngleLimits(data);
    return std::min(r.max, -r.min);  // symmetric bound for controllers / UI
}

namespace {

ComposedVehicleModel::Subsystems MakeDynamicLinearTireSubsystems(const VehicleData& data) {
    const float g = 9.81f;
    float fz_front_static = data.mass * g * data.b / data.wheelbase;
    float fz_rear_static  = data.mass * g * data.a / data.wheelbase;

    float coeff = data.tire.enabled ? data.tire.cornering_stiffness_coeff : 6.0f;
    float mu    = data.tire.enabled ? data.tire.friction_coefficient      : 1.0f;

    ComposedVehicleModel::Subsystems subsystems;
    subsystems.tire_front = std::make_unique<LinearTireModel>(
        LinearTireConfig{coeff * fz_front_static, mu});
    subsystems.tire_rear = std::make_unique<LinearTireModel>(
        LinearTireConfig{coeff * fz_rear_static, mu});

    float cog_height = data.suspension.enabled ? data.cog_height : 0.0f;
    subsystems.suspension = std::make_unique<RigidSuspensionModel>(
        RigidSuspensionConfig{data.mass, data.a, data.b, data.wheelbase, cog_height});

    subsystems.drivetrain = std::make_unique<DirectAccelDrivetrainModel>();
    subsystems.aero       = std::make_unique<SimpleAeroModel>(
        SimpleAeroConfig{data.mass, data.CA});

    const TireAngleRange lim = TireAngleLimits(data);
    if (data.steering.enabled && data.steering.use_rate_limit) {
        float max_rate = ToRadians(data.max_steering_wheel_rate) / data.steering_wheel_to_tire_angle_ratio;
        subsystems.steering = std::make_unique<RateLimitedSteeringModel>(
            RateLimitedSteeringConfig{lim.max, max_rate, lim.min});
    } else {
        subsystems.steering = std::make_unique<InstantaneousSteeringModel>(
            InstantaneousSteeringConfig{lim.max, lim.min});
    }
    return subsystems;
}

} // anonymous namespace

std::unique_ptr<IVehicleModel> MakeVehicleModel(ModelFidelity fidelity, const VehicleData& data) {
    switch (fidelity) {
        case ModelFidelity::Kinematic:
            return std::make_unique<KinematicBicycleModel>(data);
        case ModelFidelity::DynamicLinearTire:
            return std::make_unique<ComposedVehicleModel>(data, MakeDynamicLinearTireSubsystems(data));
    }
    return std::make_unique<KinematicBicycleModel>(data);  // silent-default fallback, matches project philosophy
}

} // namespace vehicle
