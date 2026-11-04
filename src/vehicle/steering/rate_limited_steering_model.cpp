#include "vehicle/steering/rate_limited_steering_model.hpp"
#include "vehicle/math_utils.hpp"

namespace vehicle {

RateLimitedSteeringModel::RateLimitedSteeringModel(const RateLimitedSteeringConfig& config)
    : config_(config) {}

float RateLimitedSteeringModel::ComputeAppliedAngle(float commanded, float current, float dt) const {
    float max_step = config_.max_rate * dt;
    float delta    = Clamp(commanded - current, -max_step, max_step);
    const float lo = (config_.min_angle == std::numeric_limits<float>::lowest()) ? -config_.max_angle : config_.min_angle;
    return Clamp(current + delta, lo, config_.max_angle);
}

} // namespace vehicle
