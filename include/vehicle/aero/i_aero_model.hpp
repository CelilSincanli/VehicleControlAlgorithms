#pragma once

namespace vehicle {

// ---------------------------------------------------------------------------
// IAeroModel
// ---------------------------------------------------------------------------
// Single responsibility: map forward speed to aerodynamic drag force.
// ---------------------------------------------------------------------------
class IAeroModel {
public:
    virtual ~IAeroModel() = default;
    virtual float ComputeDragForce(float vx) const = 0;  // [N], opposes motion
};

} // namespace vehicle
