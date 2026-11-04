#include "vehicle/steering/instantaneous_steering_model.hpp"
#include "vehicle/math_utils.hpp"

namespace vehicle {

InstantaneousSteeringModel::InstantaneousSteeringModel(const InstantaneousSteeringConfig& config)
    : config_(config) {}

float InstantaneousSteeringModel::ComputeAppliedAngle(float commanded, float current, float dt) const {
    (void)current;
    (void)dt;
    const float lo = (config_.min_angle == std::numeric_limits<float>::lowest()) ? -config_.max_angle : config_.min_angle;
    return Clamp(commanded, lo, config_.max_angle);
}

} // namespace vehicle
