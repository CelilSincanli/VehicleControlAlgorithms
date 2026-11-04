#pragma once
#include "vehicle/suspension/i_suspension_model.hpp"

namespace vehicle {

struct RigidSuspensionConfig {
    float mass       = 0.0f;  // [kg]
    float a          = 0.0f;  // distance from front axle to COG [m]
    float b          = 0.0f;  // distance from rear axle  to COG [m]
    float wheelbase  = 0.0f;  // [m]
    float cog_height = 0.0f;  // [m] — 0 reproduces today's static split exactly
};

// ---------------------------------------------------------------------------
// Rigid Suspension Model (algebraic load transfer)
// ---------------------------------------------------------------------------
// No spring/damper states — rigid chassis, classic longitudinal load-transfer
// correction on the static axle split (Gillespie, "Fundamentals of Vehicle
// Dynamics" / Wong, "Theory of Ground Vehicles"):
//   Fz_front = m·g·b/L − m·ax·h/L
//   Fz_rear  = m·g·a/L + m·ax·h/L
// Reduces exactly to the plain static split (m·g·b/L, m·g·a/L) when
// cog_height is 0 — i.e. no load transfer.
// ---------------------------------------------------------------------------
class RigidSuspensionModel : public ISuspensionModel {
public:
    explicit RigidSuspensionModel(const RigidSuspensionConfig& config = {});
    AxleLoad ComputeAxleLoads(float ax, float vx) const override;

private:
    RigidSuspensionConfig config_;
    static constexpr float kGravity = 9.81f;  // [m/s^2]
};

} // namespace vehicle
