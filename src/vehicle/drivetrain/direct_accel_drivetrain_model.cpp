#include "vehicle/drivetrain/direct_accel_drivetrain_model.hpp"

namespace vehicle {

float DirectAccelDrivetrainModel::ComputeLongitudinalForce(float u_a, float vx, float mass) const {
    (void)vx;
    return mass * u_a;
}

} // namespace vehicle
