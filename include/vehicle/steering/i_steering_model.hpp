#pragma once

namespace vehicle {

// ---------------------------------------------------------------------------
// ISteeringModel
// ---------------------------------------------------------------------------
// Single responsibility: turn a commanded tire-steer angle into the angle
// actually applied this step (travel limit, optionally a rate limit). Pure
// function — the caller supplies the previously-applied angle as `current`,
// so implementations hold no state of their own and stay trivially testable.
// Operates in tire-angle radians; the steering-wheel-to-tire-angle ratio
// conversion stays where it is today (VehicleData/caller).
// ---------------------------------------------------------------------------
class ISteeringModel {
public:
    virtual ~ISteeringModel() = default;
    virtual float ComputeAppliedAngle(float commanded, float current, float dt) const = 0;
};

} // namespace vehicle
