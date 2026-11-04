#pragma once

namespace vehicle {

// Lateral tire force output for one axle/corner.
struct TireForce {
    float fy        = 0.0f;  // lateral force [N]
    float alpha     = 0.0f;  // slip angle used to compute fy [rad]
    bool  saturated = false; // true if the friction-circle clamp was active
};

// ---------------------------------------------------------------------------
// ITireModel
// ---------------------------------------------------------------------------
// Single responsibility: map (slip angle, normal load) -> lateral tire force.
// Longitudinal/combined-slip force is out of scope for this pass.
// ---------------------------------------------------------------------------
class ITireModel {
public:
    virtual ~ITireModel() = default;
    virtual TireForce ComputeLateralForce(float alpha, float fz) const = 0;
};

} // namespace vehicle
