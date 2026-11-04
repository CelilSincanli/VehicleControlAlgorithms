#pragma once
#include "vehicle/i_vehicle_model.hpp"
#include "vehicle/vehicle_data.hpp"
#include <memory>

namespace vehicle {

// Only Kinematic and DynamicLinearTire are implemented in this pass.
// DynamicPacejkaTire/FullChassis are natural next steps (a PacejkaTireModel :
// ITireModel, a 4-corner Subsystems bundle) but are deliberately NOT added as
// enum values yet — an enum value with no corresponding case in
// MakeVehicleModel() is a worse trap than simply not having the value yet.
enum class ModelFidelity {
    Kinematic,          // KinematicBicycleModel — no lateral tire slip
    DynamicLinearTire,  // ComposedVehicleModel: LinearTireModel + RigidSuspensionModel
};

std::unique_ptr<IVehicleModel> MakeVehicleModel(ModelFidelity fidelity, const VehicleData& data);

float ToRadians(float degrees);

struct TireAngleRange { float min = 0.0f, max = 0.0f; };  // [rad], min < 0 < max
// Road-wheel travel: steering-wheel travel / ratio, further limited by
// minimum_turning_radius (authoritative): |delta| <= atan(wheelbase / radius).
TireAngleRange TireAngleLimits(const VehicleData& data);
float MaxTireAngle(const VehicleData& data);  // symmetric bound min(max, -min) [rad]

} // namespace vehicle
