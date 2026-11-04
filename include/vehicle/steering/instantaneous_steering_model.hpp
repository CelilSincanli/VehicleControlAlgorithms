#pragma once
#include "vehicle/steering/i_steering_model.hpp"
#include <limits>

namespace vehicle {

struct InstantaneousSteeringConfig {
    float max_angle = 0.0f;  // [rad] left limit
    float min_angle = std::numeric_limits<float>::lowest();  // [rad] right limit; lowest() = -max_angle
};

// ---------------------------------------------------------------------------
// Instantaneous Steering Model
// ---------------------------------------------------------------------------
// No rate limit — clamps to the travel limit only. Exactly what
// KinematicBicycleModel::Step does today via its inline max_delta clamp.
// ---------------------------------------------------------------------------
class InstantaneousSteeringModel : public ISteeringModel {
public:
    explicit InstantaneousSteeringModel(const InstantaneousSteeringConfig& config = {});
    float ComputeAppliedAngle(float commanded, float current, float dt) const override;

private:
    InstantaneousSteeringConfig config_;
};

} // namespace vehicle
