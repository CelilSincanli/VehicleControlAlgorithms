#pragma once

namespace vehicle {

// ---------------------------------------------------------------------------
// IDrivetrainModel
// ---------------------------------------------------------------------------
// Single responsibility: map a commanded acceleration + current speed to a
// net longitudinal force. Works in forces (not accelerations) so
// ComposedVehicleModel can sum drivetrain + aero once and integrate together.
// ---------------------------------------------------------------------------
class IDrivetrainModel {
public:
    virtual ~IDrivetrainModel() = default;
    virtual float ComputeLongitudinalForce(float u_a, float vx, float mass) const = 0;
};

} // namespace vehicle
