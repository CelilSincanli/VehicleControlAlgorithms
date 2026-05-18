#pragma once
#include "vehicle_data.hpp"
#include "vehicle_state.hpp"

namespace vehicle {

// ---------------------------------------------------------------------------
// Kinematic Bicycle Model
// ---------------------------------------------------------------------------
// Assumes no lateral tire slip — accurate at low/moderate speeds.
// Uses from VehicleData: wheelbase, a, b, CA.
//
// Equations (COG-referenced, rear-slip-angle β):
//   β  = atan( b / wheelbase * tan(δ) )
//   ẋ  = vx · cos(ψ + β)
//   ẏ  = vx · sin(ψ + β)
//   ψ̇  = vx / wheelbase · cos(β) · tan(δ)
//   v̇x = u_a − CA · vx²          (CA absorbs 0.5·ρ·Cd·Af / mass)
// ---------------------------------------------------------------------------
class KinematicBicycleModel {
public:
    explicit KinematicBicycleModel(const VehicleData& data);

    // Integrate one step of duration dt [s]. Returns the next VehicleState.
    VehicleState Step(const VehicleState& state,
                      const VehicleControls& controls,
                      float dt) const;

    const VehicleData& Data() const { return data_; }

private:
    VehicleData data_;
};

// ---------------------------------------------------------------------------
// Dynamic Bicycle Model
// ---------------------------------------------------------------------------
// Captures lateral tire forces and yaw dynamics.
// Uses from VehicleData: mass, a, b, CA, wheelbase.
// Cornering stiffness (Cf, Cr) and yaw inertia (Iz) are estimated from
// the loaded vehicle parameters if not explicitly provided.
//
// Vehicle-frame equations (linear tire model):
//   α_f = δ − (vy + a·r) / vx        front slip angle
//   α_r =    −(vy − b·r) / vx        rear  slip angle
//   Fy_f = Cf · α_f
//   Fy_r = Cr · α_r
//
//   v̇x = u_a − CA·vx² + vy·r
//   v̇y = (Fy_f + Fy_r) / m − vx·r
//   ṙ  = (a·Fy_f − b·Fy_r) / Iz
//
// Global-frame integration:
//   ẋ = vx·cos(ψ) − vy·sin(ψ)
//   ẏ = vx·sin(ψ) + vy·cos(ψ)
//   ψ̇ = r
//
// NOTE: Requires vx > 0; falls back to kinematic model below min_vx.
// ---------------------------------------------------------------------------
class DynamicBicycleModel {
public:
    explicit DynamicBicycleModel(const VehicleData& data);

    VehicleState Step(const VehicleState& state,
                      const VehicleControls& controls,
                      float dt) const;

    float Cf() const { return Cf_; }
    float Cr() const { return Cr_; }
    float Iz() const { return Iz_; }
    const VehicleData& Data() const { return data_; }

private:
    VehicleData data_;
    float Cf_;  // front axle cornering stiffness [N/rad]
    float Cr_;  // rear  axle cornering stiffness [N/rad]
    float Iz_;  // yaw moment of inertia           [kg·m²]

    static constexpr float kMinVx = 0.5f;  // [m/s] fallback threshold

    VehicleState KinematicFallback(const VehicleState& state,
                                   const VehicleControls& controls,
                                   float dt) const;
};

} // namespace vehicle
