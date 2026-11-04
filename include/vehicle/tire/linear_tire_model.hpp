#pragma once
#include "vehicle/tire/i_tire_model.hpp"

namespace vehicle {

struct LinearTireConfig {
    float cornering_stiffness  = 0.0f;  // C [N/rad]
    float friction_coefficient = 1.0f;  // μ — clamps |Fy| to μ·|Fz|
};

// ---------------------------------------------------------------------------
// Linear Tire Model
// ---------------------------------------------------------------------------
// Fy = C · α, clamped to the friction circle |Fy| <= μ · |Fz|. The friction
// clamp matters at large slip angles, where an unclamped linear model would
// otherwise produce unphysically large forces.
// ---------------------------------------------------------------------------
class LinearTireModel : public ITireModel {
public:
    explicit LinearTireModel(const LinearTireConfig& config = {});
    TireForce ComputeLateralForce(float alpha, float fz) const override;

private:
    LinearTireConfig config_;
};

} // namespace vehicle
