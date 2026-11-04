#pragma once
#include "vehicle/aero/i_aero_model.hpp"

namespace vehicle {

struct SimpleAeroConfig {
    float mass = 0.0f;  // [kg]
    float CA   = 0.0f;  // mass-normalised air resistance coefficient [1/m]
};

// ---------------------------------------------------------------------------
// Simple Aero Model
// ---------------------------------------------------------------------------
// Reproduces today's drag term exactly (both bicycle models compute
// ax_drag = -CA·vx·|vx|), expressed here as a force: Fx = -mass·CA·vx·|vx|.
// ---------------------------------------------------------------------------
class SimpleAeroModel : public IAeroModel {
public:
    explicit SimpleAeroModel(const SimpleAeroConfig& config = {});
    float ComputeDragForce(float vx) const override;

private:
    SimpleAeroConfig config_;
};

} // namespace vehicle
