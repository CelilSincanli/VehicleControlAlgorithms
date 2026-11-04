#pragma once
#include "vehicle/drivetrain/i_drivetrain_model.hpp"

namespace vehicle {

// ---------------------------------------------------------------------------
// Direct-Acceleration Drivetrain Model
// ---------------------------------------------------------------------------
// No engine/gearbox/brake — the control input IS the requested acceleration,
// exactly as both bicycle models treat VehicleControls::acceleration today:
//   Fx = mass · u_a
// A dedicated class (rather than inlining mass*u_a into ComposedVehicleModel)
// so a future torque-map drivetrain is a drop-in IDrivetrainModel with zero
// changes to ComposedVehicleModel.
// ---------------------------------------------------------------------------
class DirectAccelDrivetrainModel : public IDrivetrainModel {
public:
    float ComputeLongitudinalForce(float u_a, float vx, float mass) const override;
};

} // namespace vehicle
