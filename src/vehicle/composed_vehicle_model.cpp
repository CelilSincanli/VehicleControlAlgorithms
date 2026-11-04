#include "vehicle/composed_vehicle_model.hpp"
#include "vehicle/vehicle_bicycle_model.hpp"
#include <cmath>
#include <algorithm>

namespace vehicle {

ComposedVehicleModel::ComposedVehicleModel(const VehicleData& data, Subsystems subsystems)
    : data_(data), subsystems_(std::move(subsystems))
{
    // Prefer an explicit, vehicle-specific yaw_inertia (JSON override) when
    // provided. The mass*a*b fallback lumps the sprung mass into two point
    // masses at the axle stations sized to reproduce the static front/rear
    // split — a reasonable rough default, but one that converges to nearly
    // the same radius-of-gyration ratio regardless of vehicle class.
    Iz_ = (data_.yaw_inertia > 0.0f) ? data_.yaw_inertia : data_.mass * data_.a * data_.b;
}

float ComposedVehicleModel::UpdateSteering(float commanded, float dt) {
    applied_steering_ = subsystems_.steering->ComputeAppliedAngle(commanded, applied_steering_, dt);
    return applied_steering_;
}

AxleLoad ComposedVehicleModel::UpdateAxleLoads(float ax_prev, float vx) const {
    // Uses the PREVIOUS step's ax rather than solving an ax/Fz algebraic loop
    // within this step — a one-step lag that also approximates a real
    // suspension's physical lag behind commanded acceleration.
    return subsystems_.suspension->ComputeAxleLoads(ax_prev, vx);
}

void ComposedVehicleModel::ComputeTireForces(float delta, const VehicleState& s, const AxleLoad& loads) {
    const float vx = s.speed;
    const float vy = s.lateral_speed;
    const float r  = s.yaw_rate;
    const float a  = data_.a;
    const float b  = data_.b;

    float alpha_f =  delta - (vy + a * r) / vx;
    float alpha_r =        - (vy - b * r) / vx;

    TireForce ff = subsystems_.tire_front->ComputeLateralForce(alpha_f, loads.fz_front);
    TireForce fr = subsystems_.tire_rear->ComputeLateralForce(alpha_r, loads.fz_rear);

    fy_front_        = ff.fy;
    fy_rear_         = fr.fy;
    alpha_front_     = alpha_f;
    alpha_rear_      = alpha_r;
    front_saturated_ = ff.saturated;
    rear_saturated_  = fr.saturated;
}

float ComposedVehicleModel::ComputeDrivetrainForce(float u_a, float vx) const {
    return subsystems_.drivetrain->ComputeLongitudinalForce(u_a, vx, data_.mass);
}

float ComposedVehicleModel::ComputeAeroForce(float vx) const {
    return subsystems_.aero->ComputeDragForce(vx);
}

VehicleState ComposedVehicleModel::IntegrateDynamics(const VehicleState& s, float fx_total, float delta, float dt) const {
    const float vx  = s.speed;
    const float vy  = s.lateral_speed;   // at the CoG
    const float r   = s.yaw_rate;
    const float psi = s.heading;
    const float cd  = std::cos(delta);
    const float sd  = std::sin(delta);

    // Front tire force acts in the wheel frame: project it with the steering
    // angle (cornering drag on vx, cos(delta) share on vy and yaw).
    float dvx = (fx_total - fy_front_ * sd) / data_.mass + vy * r;
    float dvy = (fy_front_ * cd + fy_rear_) / data_.mass - vx * r;
    float dr  = (data_.a * fy_front_ * cd - data_.b * fy_rear_) / Iz_;

    VehicleState next = s;  // copy-then-mutate
    next.speed         = std::max(0.0f, vx + dvx * dt);
    next.lateral_speed = vy + dvy * dt;
    next.yaw_rate      = r  + dr  * dt;
    next.heading       = psi + r * dt;
    // (x, y) is the rear-axle centre (the reference every controller and the
    // renderer use); its lateral velocity is the CoG's minus b*r.
    const float vy_rear = vy - data_.b * r;
    next.x = s.x + (vx * std::cos(psi) - vy_rear * std::sin(psi)) * dt;
    next.y = s.y + (vx * std::sin(psi) + vy_rear * std::cos(psi)) * dt;
    return next;
}

bool ComposedVehicleModel::GetLastTireDiagnostics(TireDiagnostics& out) const {
    out.alpha_front      = alpha_front_;
    out.alpha_rear       = alpha_rear_;
    out.fy_front         = fy_front_;
    out.fy_rear          = fy_rear_;
    out.applied_steering = applied_steering_;
    out.front_saturated  = front_saturated_;
    out.rear_saturated   = rear_saturated_;
    return true;
}

VehicleState ComposedVehicleModel::KinematicFallback(const VehicleState& s,
                                                     const VehicleControls& u,
                                                     float dt) const {
    KinematicBicycleModel kin(data_);
    return kin.Step(s, u, dt);
}

VehicleState ComposedVehicleModel::SubStep(const VehicleState& s, const VehicleControls& u, float dt) {
    // The actuator always runs first, also in the low-speed fallback, so the
    // rate limit holds at every speed and the wheel angle is continuous.
    const float delta = UpdateSteering(u.steering_angle, dt);

    VehicleState next;
    if (s.speed < kMinVx) {
        VehicleControls applied = u;
        applied.steering_angle = delta;
        next = KinematicFallback(s, applied, dt);
        last_ax_ = (next.speed - s.speed) / std::max(dt, 1e-6f);
    } else {
        AxleLoad loads = UpdateAxleLoads(last_ax_, s.speed);
        ComputeTireForces(delta, s, loads);
        float fx_drive = ComputeDrivetrainForce(u.acceleration, s.speed);
        float fx_aero  = ComputeAeroForce(s.speed);
        next = IntegrateDynamics(s, fx_drive + fx_aero, delta, dt);
        last_ax_ = (next.speed - s.speed) / std::max(dt, 1e-6f);
    }
    next.steering_angle = delta;
    return next;
}

VehicleState ComposedVehicleModel::Step(const VehicleState& s,
                                        const VehicleControls& u,
                                        float dt)
{
    if (!actuator_seeded_) {  // continue from the wheel angle the previous model left
        applied_steering_ = s.steering_angle;
        actuator_seeded_  = true;
    }
    const int   n = std::max(1, static_cast<int>(std::ceil(dt / kMaxSubstep)));
    const float h = dt / static_cast<float>(n);
    VehicleState cur = s;
    for (int i = 0; i < n; ++i) cur = SubStep(cur, u, h);
    return cur;
}

} // namespace vehicle
