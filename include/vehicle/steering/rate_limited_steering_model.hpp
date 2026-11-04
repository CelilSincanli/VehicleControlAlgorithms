#pragma once
#include "vehicle/steering/i_steering_model.hpp"
#include <limits>

namespace vehicle {

struct RateLimitedSteeringConfig {
    float max_angle = 0.0f;  // [rad]
    float max_rate  = 0.0f;  // [rad/s]
    float min_angle = std::numeric_limits<float>::lowest();  // [rad] right limit; lowest() = -max_angle
};

// ---------------------------------------------------------------------------
// Rate-Limited Steering Model
// ---------------------------------------------------------------------------
// Activates VehicleData::max_steering_wheel_rate — parsed by VehicleLoader
// since the original field list, but never read by any model until now:
//   delta_max = max_rate · dt
//   applied   = clamp(current + clamp(commanded − current, −delta_max, delta_max),
//                      −max_angle, max_angle)
// ---------------------------------------------------------------------------
class RateLimitedSteeringModel : public ISteeringModel {
public:
    explicit RateLimitedSteeringModel(const RateLimitedSteeringConfig& config = {});
    float ComputeAppliedAngle(float commanded, float current, float dt) const override;

private:
    RateLimitedSteeringConfig config_;
};

} // namespace vehicle
