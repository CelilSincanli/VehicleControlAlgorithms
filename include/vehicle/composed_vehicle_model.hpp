#pragma once
#include "vehicle/i_vehicle_model.hpp"
#include "vehicle/vehicle_data.hpp"
#include "vehicle/tire/i_tire_model.hpp"
#include "vehicle/suspension/i_suspension_model.hpp"
#include "vehicle/drivetrain/i_drivetrain_model.hpp"
#include "vehicle/aero/i_aero_model.hpp"
#include "vehicle/steering/i_steering_model.hpp"
#include <memory>

namespace vehicle {

// ---------------------------------------------------------------------------
// ComposedVehicleModel
// ---------------------------------------------------------------------------
// Assembles a full vehicle model out of independently swappable subsystems.
// Depends only on the Ixxx abstractions above — VehicleModelFactory decides
// which concrete classes go into Subsystems; this class never names one.
// Uses vehicle-frame dynamic-bicycle equations (slip angles, linear tire
// forces, yaw-plane integration), with tire/suspension/drivetrain/aero/
// steering each delegated to a subsystem instead of inlined.
// ---------------------------------------------------------------------------
class ComposedVehicleModel : public IVehicleModel {
public:
    struct Subsystems {
        std::unique_ptr<ITireModel>       tire_front;
        std::unique_ptr<ITireModel>       tire_rear;
        std::unique_ptr<ISuspensionModel> suspension;
        std::unique_ptr<IDrivetrainModel> drivetrain;
        std::unique_ptr<IAeroModel>       aero;
        std::unique_ptr<ISteeringModel>   steering;
    };

    ComposedVehicleModel(const VehicleData& data, Subsystems subsystems);

    VehicleState Step(const VehicleState& state,
                      const VehicleControls& controls,
                      float dt) override;

    bool GetLastTireDiagnostics(TireDiagnostics& out) const override;

    const VehicleData& Data() const { return data_; }

private:
    VehicleData      data_;
    Subsystems       subsystems_;

    float applied_steering_ = 0.0f;  // last applied tire angle [rad] — fed back in as `current`
    bool  actuator_seeded_  = false; // applied_steering_ taken from the incoming state on the
                                     // first Step(), so a rebuilt model keeps the wheel angle
    float last_ax_          = 0.0f;  // previous step's longitudinal accel [m/s^2], feeds suspension
    float fy_front_         = 0.0f;
    float fy_rear_          = 0.0f;
    float alpha_front_      = 0.0f;
    float alpha_rear_       = 0.0f;
    bool  front_saturated_  = false;
    bool  rear_saturated_   = false;
    float Iz_               = 0.0f;  // yaw inertia [kg*m^2] — from VehicleData::yaw_inertia
                                     // if provided, else the mass*a*b estimate

    static constexpr float kMinVx = 0.5f;  // [m/s] — fallback threshold below which the
                                            // linear-tire slip-angle equations are singular
    static constexpr float kMaxSubstep = 0.002f;  // [s] — explicit Euler on the tire dynamics is only
                                                   // stable for dt < 2/lambda, lambda ~ (Cf+Cr)/(m vx):
                                                   // ~11 ms at 0.5 m/s for Car 01, so integrate in <= 2 ms steps

    float        UpdateSteering(float commanded, float dt);
    AxleLoad     UpdateAxleLoads(float ax_prev, float vx) const;
    void         ComputeTireForces(float delta, const VehicleState& s, const AxleLoad& loads);
    float        ComputeDrivetrainForce(float u_a, float vx) const;
    float        ComputeAeroForce(float vx) const;
    VehicleState IntegrateDynamics(const VehicleState& s, float fx_total, float delta, float dt) const;
    VehicleState SubStep(const VehicleState& s, const VehicleControls& u, float dt);
    VehicleState KinematicFallback(const VehicleState& s, const VehicleControls& u, float dt) const;
};

} // namespace vehicle
