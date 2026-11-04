// Dependency-free smoke tests for the layered vehicle model architecture.
// No test framework — a handful of plain assertions, matching the project's
// existing "no external deps beyond the vendored GUI libs" philosophy.
// Build/run: see CMakeLists.txt's `vehicle_model_tests` target.
//   ./vehicle_model_tests            (exit 0 = all pass, 1 = any failure)
//   ctest -R vehicle_model_tests --output-on-failure

#include "vehicle/vehicle_data.hpp"
#include "vehicle/vehicle_state.hpp"
#include "vehicle/vehicle_bicycle_model.hpp"
#include "vehicle/vehicle_model_factory.hpp"
#include "vehicle/tire/linear_tire_model.hpp"
#include "vehicle/aero/simple_aero_model.hpp"
#include "vehicle/drivetrain/direct_accel_drivetrain_model.hpp"
#include "vehicle/steering/rate_limited_steering_model.hpp"

#include <cstdio>
#include <cmath>
#include <algorithm>

namespace {

int g_failures = 0;

void Check(bool cond, const char* what) {
    if (cond) std::printf("[ OK ] %s\n", what);
    else      { std::fprintf(stderr, "[FAIL] %s\n", what); ++g_failures; }
}

bool NearlyEqual(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

VehicleData MakeCarLikeVehicleData() {
    VehicleData vd;
    vd.mass                               = 1500.0f;
    vd.wheelbase                          = 2.6f;
    vd.a                                  = 1.1f;
    vd.b                                  = 1.5f;
    vd.CA                                 = 0.00025f;
    vd.max_steering_wheel_angle           = 500.0f;
    vd.min_steering_wheel_angle           = -500.0f;
    vd.max_steering_wheel_rate            = 200.0f;
    vd.steering_wheel_to_tire_angle_ratio = 15.5f;
    return vd;
}

// Straight-line case: with zero steering, heading/y must stay exactly 0
// regardless of the model's internal formulation — a cheap, exact-equality
// confirmation that adding the IVehicleModel base changed no arithmetic in
// KinematicBicycleModel::Step.
void TestKinematicRegression() {
    VehicleData vd = MakeCarLikeVehicleData();
    vehicle::KinematicBicycleModel model(vd);
    vehicle::VehicleState state{};
    vehicle::VehicleControls ctrl;
    ctrl.steering_angle = 0.0f;
    ctrl.acceleration   = 1.0f;
    const float dt = 0.1f;

    for (int i = 0; i < 5; ++i) state = model.Step(state, ctrl, dt);

    Check(state.heading == 0.0f, "Kinematic: heading stays exactly 0 with zero steering");
    Check(state.y == 0.0f,       "Kinematic: y stays exactly 0 with zero steering");
    Check(state.x > 0.0f,        "Kinematic: x advances under positive acceleration");
    Check(state.speed > 0.0f,    "Kinematic: speed increases under positive acceleration");
}

void TestFactoryMatchesDirectKinematic() {
    VehicleData vd = MakeCarLikeVehicleData();

    vehicle::KinematicBicycleModel direct(vd);
    auto viaFactory = vehicle::MakeVehicleModel(vehicle::ModelFidelity::Kinematic, vd);

    vehicle::VehicleState sDirect{}, sFactory{};
    bool allMatch = true;
    for (int i = 0; i < 50; ++i) {
        vehicle::VehicleControls ctrl;
        ctrl.steering_angle = 0.2f * std::sin(0.1f * (float)i);
        ctrl.acceleration   = 0.5f;
        sDirect  = direct.Step(sDirect, ctrl, 0.05f);
        sFactory = viaFactory->Step(sFactory, ctrl, 0.05f);
        if (sDirect.x != sFactory.x || sDirect.y != sFactory.y ||
            sDirect.heading != sFactory.heading || sDirect.speed != sFactory.speed) {
            allMatch = false;
            break;
        }
    }
    Check(allMatch, "MakeVehicleModel(Kinematic) matches direct KinematicBicycleModel bit-for-bit over 50 steps");
}

void TestLinearTireModel() {
    vehicle::LinearTireModel unsaturated(vehicle::LinearTireConfig{60000.0f, 0.85f});
    // Fy = -60000 * 0.05 = -3000 N; clamp bound = 0.85*10000 = 8500 N (not hit)
    auto out1 = unsaturated.ComputeLateralForce(-0.05f, 10000.0f);
    Check(NearlyEqual(out1.fy, -3000.0f, 1.0f), "LinearTireModel: unsaturated case matches Fy = C*alpha");

    vehicle::LinearTireModel saturated(vehicle::LinearTireConfig{30000.0f, 0.9f});
    // Fy_raw = -30000 * 0.3 = -9000 N; clamp bound = 0.9*5000 = 4500 N -> clamps to -4500 N
    auto out2 = saturated.ComputeLateralForce(-0.3f, 5000.0f);
    Check(NearlyEqual(out2.fy, -4500.0f, 1.0f), "LinearTireModel: friction-circle clamp engages past the limit");
}

void TestSimpleAeroModel() {
    vehicle::SimpleAeroModel aero(vehicle::SimpleAeroConfig{8300.0f, 0.000394f});
    // Fx = -mass*CA*vx*|vx| = -8300 * 0.000394 * 20 * 20 = -1308.08 N
    float f1 = aero.ComputeDragForce(20.0f);
    Check(NearlyEqual(f1, -1308.08f, 1.0f), "SimpleAeroModel: forward drag matches -mass*CA*vx^2");

    // Moving in reverse (vx < 0): drag must still oppose the direction of travel.
    float f2 = aero.ComputeDragForce(-10.0f);
    Check(NearlyEqual(f2, 327.02f, 1.0f), "SimpleAeroModel: reverse motion drag opposes travel (sign flips)");
}

void TestDirectAccelDrivetrainModel() {
    vehicle::DirectAccelDrivetrainModel dt;
    Check(NearlyEqual(dt.ComputeLongitudinalForce(2.0f, 0.0f, 1500.0f), 3000.0f, 0.01f),
          "DirectAccelDrivetrainModel: Fx = mass*u_a (Car 01 accel case)");
    Check(NearlyEqual(dt.ComputeLongitudinalForce(-1.5f, 0.0f, 8300.0f), -12450.0f, 0.01f),
          "DirectAccelDrivetrainModel: Fx = mass*u_a (Bus 01 braking case)");
}

void TestRateLimitedSteering() {
    const float maxAngle = 1.0f;  // [rad]
    const float maxRate  = 1.0f;  // [rad/s]
    const float dt       = 0.1f;  // max step per call = 0.1 rad
    vehicle::RateLimitedSteeringModel steer(vehicle::RateLimitedSteeringConfig{maxAngle, maxRate});

    float current  = 0.0f;
    bool  allMatch = true;
    auto  step = [&](float commanded) {
        float maxStep  = maxRate * dt;
        float delta    = std::max(-maxStep, std::min(maxStep, commanded - current));
        float expected = std::max(-maxAngle, std::min(maxAngle, current + delta));
        float actual   = steer.ComputeAppliedAngle(commanded, current, dt);
        if (!NearlyEqual(actual, expected, 1e-5f)) allMatch = false;
        current = actual;
    };

    for (int i = 0; i < 6; ++i) step(0.5f);   // ramp up then settle at 0.5
    for (int i = 0; i < 3; ++i) step(-1.0f);  // ramp back down

    Check(allMatch, "RateLimitedSteeringModel: clamps rate correctly on ramp-up, settle, and reversal");
}

// Covers the yaw_inertia override added to VehicleData: ComposedVehicleModel
// should use it when provided (yaw_inertia > 0) and fall back to the
// mass*a*b two-point-mass estimate otherwise.
void TestYawInertiaOverride() {
    VehicleData base = MakeCarLikeVehicleData();
    float massAB = base.mass * base.a * base.b;  // the mass*a*b fallback value

    VehicleData dataFallback = base;               // yaw_inertia stays 0.0 -> fallback
    VehicleData dataExplicitSame = base;
    dataExplicitSame.yaw_inertia = massAB;         // should reproduce the fallback exactly
    VehicleData dataDifferent = base;
    dataDifferent.yaw_inertia = massAB * 2.0f;     // meaningfully different -> response should differ

    auto stepN = [](vehicle::IVehicleModel& model, int n) {
        vehicle::VehicleState s{};
        s.speed = 5.0f;  // above kMinVx so the dynamic path (not the kinematic fallback) runs
        vehicle::VehicleControls ctrl;
        ctrl.steering_angle = 0.15f;
        ctrl.acceleration   = 0.0f;
        for (int i = 0; i < n; ++i) s = model.Step(s, ctrl, 0.02f);
        return s;
    };

    auto fallbackModel = vehicle::MakeVehicleModel(vehicle::ModelFidelity::DynamicLinearTire, dataFallback);
    auto sameModel      = vehicle::MakeVehicleModel(vehicle::ModelFidelity::DynamicLinearTire, dataExplicitSame);
    auto diffModel      = vehicle::MakeVehicleModel(vehicle::ModelFidelity::DynamicLinearTire, dataDifferent);

    vehicle::VehicleState sFallback = stepN(*fallbackModel, 20);
    vehicle::VehicleState sSame     = stepN(*sameModel, 20);
    vehicle::VehicleState sDiff     = stepN(*diffModel, 20);

    Check(sFallback.yaw_rate == sSame.yaw_rate && sFallback.heading == sSame.heading,
          "ComposedVehicleModel: explicit yaw_inertia == mass*a*b reproduces the fallback bit-for-bit");
    Check(!NearlyEqual(sFallback.yaw_rate, sDiff.yaw_rate, 1e-4f),
          "ComposedVehicleModel: a different yaw_inertia measurably changes yaw-rate response");
}

} // namespace

int main() {
    TestKinematicRegression();
    TestFactoryMatchesDirectKinematic();
    TestLinearTireModel();
    TestSimpleAeroModel();
    TestDirectAccelDrivetrainModel();
    TestRateLimitedSteering();
    TestYawInertiaOverride();

    std::printf("\n%d failure(s)\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
