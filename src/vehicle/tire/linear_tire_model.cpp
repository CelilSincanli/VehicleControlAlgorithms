#include "vehicle/tire/linear_tire_model.hpp"
#include "vehicle/math_utils.hpp"
#include <cmath>

namespace vehicle {

LinearTireModel::LinearTireModel(const LinearTireConfig& config) : config_(config) {}

TireForce LinearTireModel::ComputeLateralForce(float alpha, float fz) const {
    float fy_max = config_.friction_coefficient * std::abs(fz);
    float fy_raw = config_.cornering_stiffness * alpha;

    TireForce out;
    out.alpha     = alpha;
    out.fy        = Clamp(fy_raw, -fy_max, fy_max);
    out.saturated = std::abs(fy_raw) > fy_max;
    return out;
}

} // namespace vehicle
