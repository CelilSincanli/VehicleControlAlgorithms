#pragma once
#include "vehicle/vehicle_state.hpp"

namespace vehicle {

// Diagnostics from the most recent Step(), for models with a tire/actuator
// subsystem. Populated by ComposedVehicleModel; models with no such
// subsystem (e.g. KinematicBicycleModel) just keep GetLastTireDiagnostics()'s
// default "unsupported" return.
struct TireDiagnostics {
    float alpha_front      = 0.0f;  // front slip angle [rad]
    float alpha_rear       = 0.0f;  // rear slip angle [rad]
    float fy_front         = 0.0f;  // front lateral tire force [N]
    float fy_rear          = 0.0f;  // rear lateral tire force [N]
    float applied_steering = 0.0f;  // actual tire angle after actuator clamp/rate-limit [rad]
    bool  front_saturated  = false;
    bool  rear_saturated   = false;
};

// ---------------------------------------------------------------------------
// IVehicleModel
// ---------------------------------------------------------------------------
// Common contract for every vehicle dynamics model (kinematic, dynamic, and
// future higher-fidelity compositions). Mirrors path_tracking::IPathTrackingAlgorithm
// on purpose: a tiny interface the simulation loop/UI depend on instead of a
// concrete model class, so vehicle model fidelity can be swapped the same
// way path-tracking algorithms already are.
// ---------------------------------------------------------------------------
class IVehicleModel {
public:
    virtual ~IVehicleModel() = default;

    // Integrate one step of duration dt [s] and return the next VehicleState.
    // Non-const: a composed model tracks genuine subsystem state between
    // steps (last applied steering angle, previous-step longitudinal accel).
    virtual VehicleState Step(const VehicleState& state,
                              const VehicleControls& controls,
                              float dt) = 0;

    // Optional: fills `out` with diagnostics from the last Step() and
    // returns true, or returns false if this model has no tire/actuator
    // subsystem to report on.
    virtual bool GetLastTireDiagnostics(TireDiagnostics& out) const { (void)out; return false; }
};

} // namespace vehicle
