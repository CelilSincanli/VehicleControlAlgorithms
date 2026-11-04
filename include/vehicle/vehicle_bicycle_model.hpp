#pragma once
#include "vehicle_data.hpp"
#include "vehicle_state.hpp"
#include "i_vehicle_model.hpp"

namespace vehicle {

// ---------------------------------------------------------------------------
// Kinematic Bicycle Model
// ---------------------------------------------------------------------------
// Assumes no lateral tire slip — accurate at low/moderate speeds. The
// steering is ideal: the commanded angle is applied instantly, clamped to the
// vehicle's tire-angle limits (vehicle::TireAngleLimits).
// Uses from VehicleData: wheelbase, CA, steering limits.
//
// Equations (rear-axle referenced, x/y = rear-axle centre):
//   ẋ  = vx · cos(ψ)
//   ẏ  = vx · sin(ψ)
//   ψ̇  = vx / wheelbase · tan(δ)
//   v̇x = u_a − CA · vx·|vx|      (CA absorbs 0.5·ρ·Cd·Af / mass)
// ---------------------------------------------------------------------------
class KinematicBicycleModel : public IVehicleModel {
public:
    explicit KinematicBicycleModel(const VehicleData& data);

    // Integrate one step of duration dt [s]. Returns the next VehicleState.
    VehicleState Step(const VehicleState& state,
                      const VehicleControls& controls,
                      float dt) override;

    const VehicleData& Data() const { return data_; }

private:
    VehicleData data_;
};

} // namespace vehicle
