#include "vehicle/suspension/rigid_suspension_model.hpp"
#include <cmath>
#include <algorithm>

namespace vehicle {

RigidSuspensionModel::RigidSuspensionModel(const RigidSuspensionConfig& config) : config_(config) {}

AxleLoad RigidSuspensionModel::ComputeAxleLoads(float ax, float vx) const {
    (void)vx;  // unused today — kept for interface symmetry with future (speed-dependent) models

    const float m = config_.mass;
    const float a = config_.a;
    const float b = config_.b;
    const float L = config_.wheelbase;
    if (std::abs(L) < 1e-6f) return {};

    float transfer = m * ax * config_.cog_height / L;

    AxleLoad load;
    load.fz_front = std::max(0.0f, m * kGravity * b / L - transfer);
    load.fz_rear  = std::max(0.0f, m * kGravity * a / L + transfer);
    return load;
}

} // namespace vehicle
