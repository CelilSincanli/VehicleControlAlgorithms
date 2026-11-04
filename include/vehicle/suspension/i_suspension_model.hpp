#pragma once

namespace vehicle {

// Per-axle normal load output.
struct AxleLoad {
    float fz_front = 0.0f;  // [N]
    float fz_rear  = 0.0f;  // [N]
};

// ---------------------------------------------------------------------------
// ISuspensionModel
// ---------------------------------------------------------------------------
// Single responsibility: map longitudinal acceleration + speed to per-axle
// normal load (load transfer). Roll/pitch dynamics are out of scope.
// ---------------------------------------------------------------------------
class ISuspensionModel {
public:
    virtual ~ISuspensionModel() = default;
    virtual AxleLoad ComputeAxleLoads(float ax, float vx) const = 0;
};

} // namespace vehicle
