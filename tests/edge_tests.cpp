// Edge-case and regression checks for the audit findings (B01-B30).
// Each check asserts documented behaviour; a [FAIL] is a defect.
// Special modes (crash probes, run as separate ctest entries):
//   edge_tests mpc_h0 | mppi_t0

#include "map/map_loader.hpp"
#include "vehicle/vehicle_loader.hpp"
#include "vehicle/vehicle_model_factory.hpp"
#include "vehicle/composed_vehicle_model.hpp"
#include "vehicle/vehicle_bicycle_model.hpp"
#include "path_tracking/pure_pursuit/pure_pursuit.hpp"
#include "path_tracking/lqr/lqr.hpp"
#include "path_tracking/mppi/mppi.hpp"
#include "path_tracking/mppi/mppi_loader.hpp"
#include "path_tracking/mpc/mpc.hpp"
#include "path_tracking/config_reader.hpp"
#include "sim/sim_rules.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace path_tracking;
namespace fs = std::filesystem;

static int g_fail = 0, g_pass = 0;
static void Check(const char* id, bool ok, const char* what, const std::string& detail = "") {
    std::printf("[%s] %-7s %s%s%s\n", ok ? " OK " : "FAIL", id, what, detail.empty() ? "" : "  -- ", detail.c_str());
    ok ? ++g_pass : ++g_fail;
}
static std::string F(const char* fmt, double a, double b = 0, double c = 0) {
    char buf[256]; std::snprintf(buf, sizeof buf, fmt, a, b, c); return buf;
}
static const std::string REPO = REPO_ROOT;
static const std::string TMP = (fs::temp_directory_path() / "vca_edge_tests").string();

static VehicleData Load(const char* file) { VehicleLoader l; l.Load(REPO + "/data/vehicle/" + file); return l.GetVehicle(); }
static VehicleData Car() { return Load("vehicle02.json"); }
static VehicleData Bus() { return Load("vehicle01.json"); }

static std::vector<Point2D> Arc(float R, int dir, int n = 200, float ds = 0.5f) {
    std::vector<Point2D> p;
    for (int i = 0; i < n; ++i) { float th = static_cast<float>(i) * ds / R; p.push_back({R * std::sin(th), static_cast<float>(dir) * R * (1 - std::cos(th))}); }
    return p;
}
static std::vector<Point2D> Line(int n, float ds = 1.0f) {
    std::vector<Point2D> p; for (int i = 0; i < n; ++i) p.push_back({static_cast<float>(i) * ds, 0}); return p;
}

// ---------------------------------------------------------------- controllers
static void T_LqrFeedforwardSign() {                       // B01
    for (int dir : {+1, -1}) {
        auto path = Arc(20.0f, dir);
        LqrConfig cfg; cfg.wheelbase = 2.6f; cfg.max_speed = 3.0f; cfg.max_delta = 0.56f; cfg.max_delta_rate = 0.23f;
        Lqr lqr(cfg); lqr.SetPath(path);
        vehicle::VehicleState s{};
        int i = 10;
        s.x = path[i].x; s.y = path[i].y; s.speed = 3.0f;
        s.heading = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
        float d = lqr.ComputeSteering(s);
        Check(dir > 0 ? "LQR-1a" : "LQR-1b", dir > 0 ? d > 0 : d < 0,
              dir > 0 ? "LQR on a LEFT arc, on-path & aligned: steers left" : "LQR on a RIGHT arc, on-path & aligned: steers right",
              F("delta = %+.4f rad", d));
    }
}

static void T_LqrNoDerivativeKick() {                      // B17
    LqrConfig cfg; cfg.wheelbase = 2.6f; cfg.max_speed = 3.0f; cfg.max_delta = 0.56f; cfg.max_delta_rate = 0.23f;
    Lqr lqr(cfg); lqr.SetPath(Line(300));
    vehicle::VehicleState s{}; s.speed = 3.0f; s.x = 10; s.y = 0.5f;
    float d1 = lqr.ComputeSteering(s), d2 = lqr.ComputeSteering(s);
    // Tiny differences come from the warm-started DARE converging to its
    // threshold; the old finite-difference kick was ~0.19 rad.
    Check("LQR-2", std::abs(d1 - d2) < 0.01f, "LQR: same pose twice gives the same command (no derivative kick)",
          F("delta1 = %+.4f, delta2 = %+.4f rad", d1, d2));
}

static void T_LqrRespectsActuator() {                      // B07 / B17
    LqrConfig cfg; cfg.wheelbase = 2.6f; cfg.max_speed = 3.0f; cfg.max_delta = 0.56f; cfg.max_delta_rate = 0.23f;
    Lqr lqr(cfg); lqr.SetPath(Line(300));
    vehicle::VehicleState s{}; s.speed = 3.0f; s.x = 10; s.y = 3.0f;   // 3 m off: wants a big correction
    float d = lqr.ComputeSteering(s);
    Check("LQR-3", std::abs(d) <= cfg.max_delta_rate * cfg.time_step + 1e-5f,
          "LQR command moves the wheel no faster than the actuator rate",
          F("|delta| = %.4f rad, rate x dt = %.4f rad", std::abs(d), cfg.max_delta_rate * cfg.time_step));
}

static void T_MppiUsesRealSpeed() {                        // B09
    auto path = Arc(15.0f, +1);
    MppiConfig cfg; cfg.wheelbase = 2.6f; cfg.max_speed = 5.0f; cfg.max_delta = 0.56f;
    Mppi a(cfg), b(cfg); a.SetPath(path); b.SetPath(path);
    vehicle::VehicleState s{}; s.y = 1.0f;
    s.speed = 0.5f;  float da = a.ComputeSteering(s);
    s.speed = 12.0f; float db = b.ComputeSteering(s);
    Check("MPPI-1", da != db, "MPPI output depends on the real vehicle speed", F("delta@0.5 = %+.5f, delta@12 = %+.5f", da, db));
}

static void T_MppiLambdaZero() {                           // B16
    MppiConfig cfg; cfg.wheelbase = 2.6f; cfg.lambda = 0.0f; cfg.max_delta = 0.56f;
    Mppi m(cfg); m.SetPath(Line(300));
    vehicle::VehicleState s{}; s.x = 5; s.speed = 3;
    float d = 0; for (int k = 0; k < 5; ++k) d = m.ComputeSteering(s);
    Check("MPPI-2", std::isfinite(d) && std::abs(d) < 0.5f * cfg.max_delta, "MPPI with lambda = 0 on a straight path does not saturate",
          F("delta = %+.3f rad", d));
}

static void T_MppiRateFeasible() {                         // B09
    MppiConfig cfg; cfg.wheelbase = 2.6f; cfg.max_delta = 0.56f; cfg.max_delta_rate = 0.23f;
    Mppi m(cfg); m.SetPath(Arc(10.0f, +1));
    vehicle::VehicleState s{}; s.y = -3.0f; s.speed = 3;
    float d = m.ComputeSteering(s);
    Check("MPPI-3", std::abs(d) <= cfg.max_delta_rate * cfg.control_dt + 1e-5f,
          "MPPI command is feasible for the actuator rate in one control period", F("|delta| = %.4f rad", std::abs(d)));
}

static void T_MpcDeltaRateZero() {
    MpcConfig cfg; cfg.wheelbase = 2.6f; cfg.max_delta = 0.56f; cfg.max_delta_rate = 0.0f;
    Mpc m(cfg); m.SetPath(Arc(15.0f, +1));
    vehicle::VehicleState s{}; s.y = -2.0f; s.speed = 3;
    float d = m.ComputeSteering(s);
    Check("MPC-1", std::abs(d) > 1e-4f, "MPC with an unknown (0) steering rate still steers", F("delta = %+.5f rad", d));
}

static void T_MpcCommandFeasible() {                       // B03
    VehicleData vd = Car();
    MpcConfig cfg; cfg.wheelbase = vd.wheelbase; cfg.max_delta = vehicle::MaxTireAngle(vd);
    cfg.max_delta_rate = vehicle::ToRadians(vd.max_steering_wheel_rate) / vd.steering_wheel_to_tire_angle_ratio;
    Mpc m(cfg); m.SetPath(Arc(10.0f, +1));
    vehicle::VehicleState s{}; s.y = -3.0f; s.speed = 3;
    float d = m.ComputeSteering(s);
    float feasible = cfg.max_delta_rate * cfg.control_dt;
    Check("MPC-2", std::abs(d) <= feasible + 1e-5f, "MPC first command is feasible for the actuator rate",
          F("delta = %+.4f rad, feasible = %.4f rad", d, feasible));
}

static void T_MpcSteady() {                                // B02
    VehicleData vd = Car();
    auto model = vehicle::MakeVehicleModel(vehicle::ModelFidelity::Kinematic, vd);
    MpcConfig cfg; cfg.wheelbase = vd.wheelbase; cfg.max_delta = vehicle::MaxTireAngle(vd);
    cfg.max_delta_rate = vehicle::ToRadians(vd.max_steering_wheel_rate) / vd.steering_wheel_to_tire_angle_ratio;
    Mpc m(cfg); m.SetPath(Arc(15.0f, +1, 400, 0.5f));
    vehicle::VehicleState s{}; s.speed = 3.0f; s.y = 0.5f;
    const float max_step = cfg.max_delta_rate * cfg.control_dt;
    int reversals = 0; float prev = 0, worst_lat = 0;
    for (int k = 0; k < 720; ++k) {                     // 12 s; judge the last 2 s
        float d = m.ComputeSteering(s);
        const float rate_step = d - s.steering_angle;
        if (k >= 600) {
            TrackingDiagnostics t; m.GetLastTrackingDiagnostics(t);
            worst_lat = std::max(worst_lat, std::abs(t.cross_track_error_m));
            if (rate_step * prev < 0 && std::abs(rate_step) > 0.5f * max_step && std::abs(prev) > 0.5f * max_step) ++reversals;
        }
        prev = rate_step;
        vehicle::VehicleControls u; u.steering_angle = d; u.acceleration = 0;
        s = model->Step(s, u, sim::kControlDt);
    }
    Check("MPC-3", worst_lat < 0.1f && reversals == 0, "MPC settles on a constant arc without full-rate steering reversals",
          F("max |CTE| over last 2 s = %.3f m, reversals = %.0f", worst_lat, reversals));
}

static void T_RebuildKeepsCursor() {                       // B13
    PurePursuitConfig cfg; cfg.wheelbase = 2.6f; cfg.lookahead_distance = 5; cfg.lookahead_gain = 0.5f;
    PurePursuit pp(cfg); pp.SetPath(Line(400, 1.0f));
    vehicle::VehicleState s{}; s.speed = 5;
    for (int k = 0; k <= 250; ++k) { s.x = (float)k; pp.ComputeSteering(s); }
    PurePursuit rebuilt(cfg); rebuilt.SetPath(Line(400, 1.0f));
    rebuilt.ComputeSteering(s);
    Check("UI-1", rebuilt.GetLookaheadPoint().x > s.x, "After a rebuild (live slider change) the lookahead stays ahead of the car",
          F("car x = %.0f, lookahead x = %.1f", s.x, rebuilt.GetLookaheadPoint().x));
    Check("PP-1", std::abs(rebuilt.GetLookaheadPoint().x - (s.x + 5 + 0.5f * 5)) < 0.05f,
          "Pure Pursuit lookahead lies exactly Ld from the rear axle (interpolated)",
          F("lookahead x = %.3f, expected %.3f", rebuilt.GetLookaheadPoint().x, s.x + 7.5));
}

// ---------------------------------------------------------------- vehicle model
static void T_LowSpeedRateLimit() {                        // B12
    VehicleData vd = Car();
    auto m = vehicle::MakeVehicleModel(vehicle::ModelFidelity::DynamicLinearTire, vd);
    vehicle::VehicleState s{}; s.speed = 0.3f;
    vehicle::VehicleControls u; u.steering_angle = 0.4f;
    auto n = m->Step(s, u, sim::kControlDt);
    float max_step = vehicle::ToRadians(vd.max_steering_wheel_rate) / vd.steering_wheel_to_tire_angle_ratio * sim::kControlDt;
    Check("VM-1", n.steering_angle <= max_step * 1.01f, "Rate limit applies below 0.5 m/s (kinematic fallback)",
          F("applied = %.4f rad, limit = %.4f rad", n.steering_angle, max_step));
    // accelerate through the 0.5 m/s handover with a constant command
    auto m2 = vehicle::MakeVehicleModel(vehicle::ModelFidelity::DynamicLinearTire, vd);
    s = {}; u.steering_angle = 0.3f; u.acceleration = 1.0f;
    float worst = 0, prev = 0;
    for (int k = 0; k < 120; ++k) { s = m2->Step(s, u, sim::kControlDt); worst = std::max(worst, std::abs(s.steering_angle - prev)); prev = s.steering_angle; }
    Check("VM-2", worst <= max_step * 1.01f, "Wheel angle is continuous across the fallback -> dynamic handover",
          F("largest per-step change = %.4f rad, limit = %.4f rad", worst, max_step));
}

static void T_ReferencePoint() {                           // B07
    VehicleData vd = Car(); vd.steering.use_rate_limit = false;
    auto dyn = vehicle::MakeVehicleModel(vehicle::ModelFidelity::DynamicLinearTire, vd);
    vehicle::VehicleState s{}; s.speed = 3.0f;
    vehicle::VehicleControls u; u.steering_angle = 0.15f; u.acceleration = vd.CA * 9.0f;
    vehicle::VehicleState prev = s;
    for (int k = 0; k < 1200; ++k) { prev = s; s = dyn->Step(s, u, 1.0f / 120.0f); }
    float slip = (std::atan2(s.y - prev.y, s.x - prev.x) - prev.heading) * 57.2958f;
    while (slip > 180) slip -= 360;
    while (slip < -180) slip += 360;
    Check("VM-3", std::abs(slip) < 1.0f, "Dynamic model: (x, y) is the rear axle (no sideways crab at 3 m/s)",
          F("velocity vs heading = %.2f deg", slip));
}

static void T_EulerAccuracy() {                            // B11
    VehicleData vd = Car(); vd.steering.use_rate_limit = false;
    std::string detail; bool all = true;
    for (float v : {0.6f, 1.0f, 2.0f}) for (float dt : {1.0f / 60.0f, 1.0f / 30.0f}) {
        auto m = vehicle::MakeVehicleModel(vehicle::ModelFidelity::DynamicLinearTire, vd);
        auto ref = vehicle::MakeVehicleModel(vehicle::ModelFidelity::DynamicLinearTire, vd);
        vehicle::VehicleState s{}, r{}; s.speed = r.speed = v;
        vehicle::VehicleControls u; u.steering_angle = 0.1f;
        for (float t = 0; t < 2.0f; t += dt) s = m->Step(s, u, dt);
        for (int k = 0; k < 2000; ++k) r = ref->Step(r, u, 0.001f);
        float err = std::abs(s.yaw_rate - r.yaw_rate) / std::max(std::abs(r.yaw_rate), 1e-3f);
        all &= err < 0.1f;
        detail += F("v=%.1f dt=%.0fms err=%.0f%%; ", v, dt * 1000, err * 100);
    }
    Check("VM-4", all, "Dynamic model accurate at frame-rate dt (vs 1 ms reference)", detail);
}

static void T_RebuildKeepsWheel() {                        // B14
    VehicleData vd = Car();
    auto a = vehicle::MakeVehicleModel(vehicle::ModelFidelity::DynamicLinearTire, vd);
    vehicle::VehicleState s{}; s.speed = 3.0f;
    vehicle::VehicleControls u; u.steering_angle = 0.3f;
    for (int k = 0; k < 120; ++k) s = a->Step(s, u, sim::kControlDt);
    const float before = s.steering_angle;
    auto b = vehicle::MakeVehicleModel(vehicle::ModelFidelity::DynamicLinearTire, vd);   // rebuilt mid-run
    s = b->Step(s, u, sim::kControlDt);
    Check("VM-5", std::abs(s.steering_angle - before) < 0.01f, "A rebuilt vehicle model keeps the wheel angle",
          F("before = %.3f, after rebuild = %.3f rad", before, s.steering_angle));
}

static void T_TurningRadiusAuthority() {                   // B27
    VehicleData bus = Bus(), car = Car();
    float b = vehicle::MaxTireAngle(bus) * 57.2958f, c = vehicle::MaxTireAngle(car) * 57.2958f;
    float eb = std::atan(bus.wheelbase / bus.minimum_turning_radius) * 57.2958f;
    float ec = std::atan(car.wheelbase / car.minimum_turning_radius) * 57.2958f;
    Check("VD-1", std::abs(b - eb) < 0.05f && std::abs(c - ec) < 0.05f,
          "Tire-angle limit follows minimum_turning_radius (atan(L/R))", F("Bus %.1f deg, Car %.1f deg", b, c));
}

// ---------------------------------------------------------------- loaders
static void T_Loaders() {                                  // B15 / B16
    fs::create_directories(TMP);
    std::string f = TMP + "/veh_missing_ratio.json";
    std::ofstream(f) << R"({"name":"NoRatio","mass":1500,"a":1.1,"b":1.5,"CA":0.00025,"wheelbase":2.6,
        "max_steering_wheel_angle":500,"wheel_tread":1.55})";
    VehicleLoader l; bool ok = l.Load(f);
    float tire_deg = vehicle::MaxTireAngle(l.GetVehicle()) * 57.2958f;
    Check("LD-1", ok && tire_deg >= 9.9f && tire_deg <= 60.1f && !l.GetWarnings().empty(),
          "Missing vehicle fields are clamped to physical values and reported",
          F("max tire angle = %.1f deg, warnings = %.0f", tire_deg, (double)l.GetWarnings().size()));

    std::string g = TMP + "/veh_truncated.json";
    std::ofstream(g) << R"({"name":"Broken","mass":15)";
    VehicleLoader l2; bool ok2 = l2.Load(g);
    Check("LD-2", !ok2 && !l2.GetError().empty(), "Truncated vehicle JSON is rejected with a reason", l2.GetError());

    std::string h = TMP + "/veh_sparse.json";
    std::ofstream(h) << R"({"name":"Sparse","mass":1500})";
    VehicleLoader l3; l3.Load(h);
    auto m = vehicle::MakeVehicleModel(vehicle::ModelFidelity::DynamicLinearTire, l3.GetVehicle());
    vehicle::VehicleState s{}; s.speed = 2.0f; vehicle::VehicleControls u; u.steering_angle = 0.1f;
    s = m->Step(s, u, 0.016f);
    Check("LD-3", std::isfinite(s.x) && std::isfinite(s.yaw_rate), "A sparse (but valid) vehicle file simulates without NaN",
          F("x = %f, yaw_rate = %f", s.x, s.yaw_rate));

    for (const char* file : {"vehicle01.json", "vehicle02.json"}) {
        VehicleLoader v; bool okv = v.Load(REPO + "/data/vehicle/" + file);
        Check("LD-5", okv && v.GetWarnings().empty(), "Shipped vehicle file loads without warnings",
              std::string(file) + (v.GetWarnings().empty() ? "" : ": " + v.GetWarnings().front()));
    }

    std::string d = TMP + "/map_truncated";
    fs::create_directories(d);
    std::ofstream(d + "/map.json") << R"({"name":"Half","world":{"width":50,"height":50},
        "start":{"x":1,"y":1},"goal":{"x":40,"y":40},"vehicle_radius":1.2,"obstacles":[]})";
    std::ofstream(d + "/path.json") << R"({"waypoints": [ {"x":1,"y":1}, {"x":40, )";
    MapLoader ml; bool okm = ml.Load(d);
    Check("LD-4", !okm && !ml.GetError().empty() && !ml.GetMap().loaded, "A truncated path.json is rejected and no half map is kept",
          ml.GetError());

    float v = 5.0f; bool okq = readFloat(R"({"max_speed": "8.0"})", "max_speed", v);
    Check("CFG-1", okq && v == 8.0f, "A quoted number in a config is parsed", F("value = %.1f", v));

    std::string c = TMP + "/mppi_bad.json";
    std::ofstream(c) << R"({"num_samples_K": -5, "lambda": "abc", "horizon_T": 0})";
    auto mc = LoadMppiConfig(c);
    Check("CFG-2", mc.num_samples_K >= tuning::kMPPISamples.lo && mc.horizon_T >= tuning::kMPPIHorizon.lo && mc.warnings.size() == 3,
          "Config loader clamps out-of-range values and reports unparsable ones",
          F("K = %.0f, T = %.0f, warnings = %.0f", mc.num_samples_K, mc.horizon_T, (double)mc.warnings.size()));
}

// ---------------------------------------------------------------- simulation rules
static void T_SimRules() {                                 // B04 / B18
    VehicleData vd = Car();
    MapData map; map.world_width = 100; map.world_height = 100; map.goal = {50, 50};
    std::vector<Point2D> path = {{0, 50}, {25, 50}, {50, 50}};
    vehicle::VehicleState s{}; s.x = 51.5f; s.y = 51.8f;     // 2.3 m from the goal, past the goal line
    auto o = sim::CheckTermination(s, vd, map, path, sim::PathLength(path), 10, 3);
    Check("TERM-1", o == sim::Outcome::Goal, "Crossing the goal line near the path ends the run (no near-miss loop)",
          sim::OutcomeName(o));
    s.x = 20; s.y = 50;
    map.obstacles.push_back(Obstacle{"o", "rectangle", 21, 49, 2, 2});
    o = sim::CheckTermination(s, vd, map, path, sim::PathLength(path), 10, 3);
    Check("TERM-2", o == sim::Outcome::Collision, "Body touching an obstacle ends the run as a collision", sim::OutcomeName(o));
    map.obstacles.clear(); s.x = 1; s.y = 99.5f;
    o = sim::CheckTermination(s, vd, map, path, sim::PathLength(path), 10, 3);
    Check("TERM-3", o == sim::Outcome::OffMap, "Body leaving the world ends the run", sim::OutcomeName(o));
    s.x = 20; s.y = 50;
    o = sim::CheckTermination(s, vd, map, path, sim::PathLength(path), 1000, 3);
    Check("TERM-4", o == sim::Outcome::Timeout, "A run that takes far too long times out", sim::OutcomeName(o));

    float a = 0, worst_jerk = 0, peak = 0;
    for (int k = 0; k < 300; ++k) {
        float n = sim::SpeedCommand(12.0f, 0.0f, a, vd, sim::kControlDt);
        worst_jerk = std::max(worst_jerk, std::abs(n - a) / sim::kControlDt); peak = std::max(peak, n); a = n;
    }
    Check("SPD-1", peak <= vd.max_acceleration + 1e-4f && worst_jerk <= vd.max_jerk + 1e-3f,
          "Speed command respects the vehicle's acceleration and jerk limits",
          F("peak %.2f m/s^2 (limit %.1f), jerk %.2f m/s^3 (limit %.1f)", peak, vd.max_acceleration, worst_jerk) + F(" / %.1f", vd.max_jerk));
}

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "mpc_h0") == 0) {
        MpcConfig c; c.horizon_N = 0; Mpc m(c); m.SetPath(Line(10));
        vehicle::VehicleState s{}; s.speed = 1; std::printf("mpc_h0 -> %f\n", m.ComputeSteering(s)); return 0;
    }
    if (argc > 1 && std::strcmp(argv[1], "mppi_t0") == 0) {
        MppiConfig c; c.horizon_T = 0; Mppi m(c); m.SetPath(Line(10));
        vehicle::VehicleState s{}; s.speed = 1; std::printf("mppi_t0 -> %f\n", m.ComputeSteering(s)); return 0;
    }
    T_LqrFeedforwardSign();
    T_LqrNoDerivativeKick();
    T_LqrRespectsActuator();
    T_MppiUsesRealSpeed();
    T_MppiLambdaZero();
    T_MppiRateFeasible();
    T_MpcDeltaRateZero();
    T_MpcCommandFeasible();
    T_MpcSteady();
    T_RebuildKeepsCursor();
    T_LowSpeedRateLimit();
    T_ReferencePoint();
    T_EulerAccuracy();
    T_RebuildKeepsWheel();
    T_TurningRadiusAuthority();
    T_Loaders();
    T_SimRules();
    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
