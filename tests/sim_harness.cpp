// Headless closed-loop harness. Runs one scenario exactly like the GUI loop
// (VehicleControlUI::InitSimulation / RebuildVehicleModel / RebuildAlgorithm /
// ApplyTuningLimits / fixed-step TickSimulation) using the project's real
// sources and the shared simulation rules (sim/sim_rules.hpp), and prints one
// JSON line of metrics.
//
// Usage: sim_harness --map DIR --vehicle JSON --algo NAME --fidelity kin|dyn
//                    [--frame-dt S] [--jitter LO,HI] [--speed MPS] [--trace CSV]
//                    [--rebuild-at S] [--switch-fidelity-at S] [--seed N] [--tag TXT]
//                    [--no-cap] [--y0 M]
//   --frame-dt / --jitter : wall-clock frame time fed to the fixed-step accumulator
//   --no-cap              : ignore the Dynamic speed cap (calibration sweeps only)
//   --y0                  : micro start offset [m] for sensitivity runs

#include "map/map_loader.hpp"
#include "vehicle/vehicle_loader.hpp"
#include "vehicle/vehicle_model_factory.hpp"
#include "path_tracking/pure_pursuit/pure_pursuit.hpp"
#include "path_tracking/pure_pursuit/pure_pursuit_loader.hpp"
#include "path_tracking/adaptive_pure_pursuit/adaptive_pure_pursuit.hpp"
#include "path_tracking/adaptive_pure_pursuit/adaptive_pure_pursuit_loader.hpp"
#include "path_tracking/lqr/lqr.hpp"
#include "path_tracking/lqr/lqr_loader.hpp"
#include "path_tracking/stanley/stanley.hpp"
#include "path_tracking/stanley/stanley_loader.hpp"
#include "path_tracking/mppi/mppi.hpp"
#include "path_tracking/mppi/mppi_loader.hpp"
#include "path_tracking/mpc/mpc.hpp"
#include "path_tracking/mpc/mpc_loader.hpp"
#include "path_tracking/path_feasibility.hpp"
#include "sim/sim_rules.hpp"
#include "tuning_limits.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace path_tracking;

struct Args {
    std::string map, vehicle, algo = "Pure Pursuit", fidelity = "kin", trace, tag;
    float frame_dt = 1.0f / 60.0f, jlo = -1, jhi = -1, speed = -1;
    float rebuild_at = -1, switch_at = -1, y0 = 0;
    int seed = 1;
    bool no_cap = false;
};

static Args Parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto v = [&]() { return std::string(argv[++i]); };
        if (k == "--map") a.map = v();
        else if (k == "--vehicle") a.vehicle = v();
        else if (k == "--algo") a.algo = v();
        else if (k == "--fidelity") a.fidelity = v();
        else if (k == "--frame-dt") a.frame_dt = std::stof(v());
        else if (k == "--jitter") { auto s = v(); auto c = s.find(','); a.jlo = std::stof(s.substr(0, c)); a.jhi = std::stof(s.substr(c + 1)); }
        else if (k == "--speed") a.speed = std::stof(v());
        else if (k == "--trace") a.trace = v();
        else if (k == "--rebuild-at") a.rebuild_at = std::stof(v());
        else if (k == "--switch-fidelity-at") a.switch_at = std::stof(v());
        else if (k == "--seed") a.seed = std::stoi(v());
        else if (k == "--tag") a.tag = v();
        else if (k == "--no-cap") a.no_cap = true;
        else if (k == "--y0") a.y0 = std::stof(v());
    }
    return a;
}

// Mirror of RebuildAlgorithm() with tune structs seeded from JSON (override
// off) and clamped like ApplyTuningLimits() (speed cap, tire-angle cap).
static std::unique_ptr<IPathTrackingAlgorithm> MakeAlgo(const std::string& name, const VehicleData& vd,
                                                       vehicle::ModelFidelity fidelity,
                                                       float speed_override, float vmax_limit,
                                                       float& max_speed_out) {
    const std::string C = std::string(CONFIG_PATH) + "/path_tracking/";
    const float dmax = std::max(tuning::kMaxDelta.lo, std::min(tuning::kMaxDelta.hi, vehicle::MaxTireAngle(vd)));
    // ActuatorRateLimit(): the vehicle's rate under Dynamic (if rate-limited), instant under Kinematic
    const bool  rate_limited = fidelity == vehicle::ModelFidelity::DynamicLinearTire &&
                               vd.steering.enabled && vd.steering.use_rate_limit && vd.max_steering_wheel_rate > 0.0f;
    const float rate = rate_limited ? vehicle::ToRadians(vd.max_steering_wheel_rate) / vd.steering_wheel_to_tire_angle_ratio
                                    : std::numeric_limits<float>::max();
    auto speed = [&](float file_value) {
        float v = speed_override > 0 ? speed_override : file_value;
        return std::max(tuning::kMaxSpeed.lo, std::min(vmax_limit, v));
    };
    if (name == "Adaptive Pure Pursuit") {
        auto f = LoadAdaptivePurePursuitConfig(C + "adaptive_pure_pursuit.json");
        AdaptivePurePursuitConfig cfg;
        cfg.min_lookahead = f.min_lookahead; cfg.max_lookahead = f.max_lookahead;
        cfg.speed_gain = f.speed_gain; cfg.curvature_gain = f.curvature_gain;
        cfg.wheelbase = vd.wheelbase; cfg.search_window = f.search_window;
        max_speed_out = speed(f.max_speed_mps);
        return std::make_unique<AdaptivePurePursuit>(cfg);
    } else if (name == "LQR") {
        auto f = LoadLqrConfig(C + "lqr.json");
        LqrConfig cfg;
        cfg.q0 = f.q0; cfg.q1 = f.q1; cfg.q2 = f.q2; cfg.q3 = f.q3; cfg.q4 = f.q4; cfg.q5 = f.q5;
        cfg.r_steering = f.r_steering; cfg.r_acceleration = f.r_acceleration; cfg.r_scale = f.r_scale;
        cfg.time_step = sim::kControlDt;
        cfg.dare_iterations = f.dare_iterations; cfg.dare_threshold = f.dare_threshold;
        cfg.wheelbase = vd.wheelbase; cfg.search_window = f.search_window;
        cfg.max_delta = dmax; cfg.max_delta_rate = rate;
        max_speed_out = speed(f.max_speed); cfg.max_speed = max_speed_out;
        return std::make_unique<Lqr>(cfg);
    } else if (name == "Stanley") {
        auto f = LoadStanleyConfig(C + "stanley.json");
        StanleyConfig cfg;
        cfg.stanley_gain = f.stanley_gain; cfg.min_speed = f.min_speed;
        cfg.max_delta = std::min(f.max_delta, dmax);
        cfg.wheelbase = vd.wheelbase; cfg.search_window = f.search_window;
        max_speed_out = speed(f.max_speed_mps); cfg.max_speed = max_speed_out;
        return std::make_unique<Stanley>(cfg);
    } else if (name == "MPPI") {
        auto f = LoadMppiConfig(C + "mppi.json");
        MppiConfig cfg;
        cfg.horizon_T = f.horizon_T; cfg.num_samples_K = f.num_samples_K; cfg.lambda = f.lambda;
        cfg.sigma_steer = f.sigma_steer; cfg.rollout_dt = f.rollout_dt;
        cfg.w_lat = f.w_lat; cfg.w_heading = f.w_heading; cfg.wheelbase = vd.wheelbase;
        cfg.search_window = f.search_window; cfg.max_delta = std::min(f.max_delta, dmax);
        cfg.rng_seed = f.rng_seed; cfg.max_delta_rate = rate; cfg.control_dt = sim::kControlDt;
        max_speed_out = speed(f.max_speed_mps); cfg.max_speed = max_speed_out;
        return std::make_unique<Mppi>(cfg);
    } else if (name == "MPC") {
        auto f = LoadMpcConfig(C + "mpc.json");
        MpcConfig cfg;
        cfg.horizon_N = f.horizon_N; cfg.iterations = f.iterations; cfg.rollout_dt = f.rollout_dt;
        cfg.w_lat = f.w_lat; cfg.w_heading = f.w_heading; cfg.w_control = f.w_control; cfg.w_rate = f.w_rate;
        cfg.wheelbase = vd.wheelbase; cfg.search_window = f.search_window;
        cfg.max_delta = std::min(f.max_delta, dmax); cfg.max_delta_rate = rate; cfg.control_dt = sim::kControlDt;
        max_speed_out = speed(f.max_speed_mps); cfg.max_speed = max_speed_out;
        return std::make_unique<Mpc>(cfg);
    }
    auto f = LoadPurePursuitConfig(C + "pure_pursuit.json");
    PurePursuitConfig cfg;
    cfg.lookahead_distance = f.lookahead_distance; cfg.lookahead_gain = f.lookahead_gain;
    cfg.wheelbase = vd.wheelbase; cfg.search_window = f.search_window;
    max_speed_out = speed(f.max_speed_mps);
    return std::make_unique<PurePursuit>(cfg);
}

static float SignedDistToPath(const std::vector<Point2D>& p, float x, float y, int& seg_out) {
    float best = 1e30f, sgn = 1; int bs = 0;
    for (size_t i = 0; i + 1 < p.size(); ++i) {
        float ax = p[i].x, ay = p[i].y, vx = p[i + 1].x - ax, vy = p[i + 1].y - ay, L2 = vx * vx + vy * vy;
        float t = L2 > 0 ? std::max(0.0f, std::min(1.0f, ((x - ax) * vx + (y - ay) * vy) / L2)) : 0.0f;
        float d = std::hypot(x - (ax + t * vx), y - (ay + t * vy));
        if (d < best) { best = d; bs = (int)i; sgn = (vx * (y - ay) - vy * (x - ax)) >= 0 ? 1.f : -1.f; }
    }
    seg_out = bs;
    return best * sgn;
}

int main(int argc, char** argv) {
    Args A = Parse(argc, argv);
    MapLoader ml;
    if (!ml.Load(A.map)) { std::printf("{\"error\":\"map: %s\"}\n", ml.GetError().c_str()); return 2; }
    VehicleLoader vl;
    if (!vl.Load(A.vehicle)) { std::printf("{\"error\":\"vehicle: %s\"}\n", vl.GetError().c_str()); return 2; }
    const VehicleData& vd = vl.GetVehicle();
    const MapData& map = ml.GetMap();
    const std::vector<Point2D>& path = ml.GetPath().waypoints;   // metres

    vehicle::VehicleState st{};
    st.x = map.start.x;
    st.y = map.start.y + A.y0;
    st.heading = std::atan2(path[1].y - path[0].y, path[1].x - path[0].x);

    auto fid = (A.fidelity == "dyn") ? vehicle::ModelFidelity::DynamicLinearTire : vehicle::ModelFidelity::Kinematic;
    auto model = vehicle::MakeVehicleModel(fid, vd);
    const auto feas = AnalyzePathFeasibility(path, vd);
    auto speedLimit = [&](vehicle::ModelFidelity f) {
        float hi = tuning::kMaxSpeed.hi;
        if (f == vehicle::ModelFidelity::DynamicLinearTire && !A.no_cap) hi = std::min(hi, feas.speed_cap_mps);
        return std::max(tuning::kMaxSpeed.lo, hi);
    };
    float maxSpeed = 3.0f;
    auto algo = MakeAlgo(A.algo, vd, fid, A.speed, speedLimit(fid), maxSpeed);
    algo->SetPath(path);
    const float path_len = sim::PathLength(path);

    std::mt19937 rng(A.seed);
    std::uniform_real_distribution<float> jit(A.jlo, A.jhi);
    FILE* tr = nullptr;
    if (!A.trace.empty()) {
        tr = std::fopen(A.trace.c_str(), "w");
        std::fprintf(tr, "t,x,y,heading,speed,vy,yaw_rate,delta_cmd,delta_applied,accel,cte\n");
    }

    float t = 0, accum = 0, a_prev = 0;
    long ticks = 0;
    sim::Outcome outcome = sim::Outcome::Running;
    double sum_cte2 = 0; float max_cte = 0; int max_seg = 0;
    long sat_ticks = 0; float max_lat_acc = 0, max_speed_seen = 0, max_abs_jerk = 0, max_accel = 0, max_brake = 0;
    double steer_tv = 0, applied_tv = 0; float prev_cmd = 0, prev_applied = 0, prev_sign = 0; int sign_changes = 0;
    double ctrl_us_sum = 0, ctrl_us_max = 0;
    bool rebuilt = false, switched = false;
    float dsteer_after_rebuild = 0, prev_cmd_before_rebuild = 0;
    const float dmax = std::max(tuning::kMaxDelta.lo, std::min(tuning::kMaxDelta.hi, vehicle::MaxTireAngle(vd)));

    while (outcome == sim::Outcome::Running) {
        float frame = (A.jlo > 0) ? jit(rng) : A.frame_dt;
        accum += std::min(frame, 0.25f);
        int steps = 0;
        while (accum >= sim::kControlDt && steps < 15 && outcome == sim::Outcome::Running) {
            accum -= sim::kControlDt; ++steps;
            const float dt = sim::kControlDt;
            if (A.rebuild_at >= 0 && !rebuilt && t >= A.rebuild_at) {   // paramsDirty_ -> RebuildAlgorithm()
                algo = MakeAlgo(A.algo, vd, fid, A.speed, speedLimit(fid), maxSpeed);
                algo->SetPath(path);
                rebuilt = true;
                prev_cmd_before_rebuild = prev_cmd;
            }
            if (A.switch_at >= 0 && !switched && t >= A.switch_at) {    // modelDirty_ -> RebuildVehicleModel()
                fid = (fid == vehicle::ModelFidelity::Kinematic) ? vehicle::ModelFidelity::DynamicLinearTire
                                                                 : vehicle::ModelFidelity::Kinematic;
                model = vehicle::MakeVehicleModel(fid, vd);
                switched = true;
                float s2;   // ApplyTuningLimits: rebuild only if the speed had to be clamped
                auto probe = MakeAlgo(A.algo, vd, fid, A.speed, speedLimit(fid), s2);
                if (s2 != maxSpeed) { algo = std::move(probe); algo->SetPath(path); maxSpeed = s2; }
                const bool rl = fid == vehicle::ModelFidelity::DynamicLinearTire && vd.steering.enabled &&
                                vd.steering.use_rate_limit && vd.max_steering_wheel_rate > 0.0f;
                algo->SetActuatorRate(rl ? vehicle::ToRadians(vd.max_steering_wheel_rate) / vd.steering_wheel_to_tire_angle_ratio
                                         : std::numeric_limits<float>::max());
            }

            auto c0 = std::chrono::steady_clock::now();
            float delta = algo->ComputeSteering(st);
            double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - c0).count();
            ctrl_us_sum += us; ctrl_us_max = std::max(ctrl_us_max, us);

            vehicle::VehicleControls ctrl;
            ctrl.steering_angle = delta;
            ctrl.acceleration   = sim::SpeedCommand(maxSpeed, st.speed, a_prev, vd, dt);
            max_abs_jerk = std::max(max_abs_jerk, std::abs(ctrl.acceleration - a_prev) / dt);
            max_accel = std::max(max_accel, ctrl.acceleration); max_brake = std::max(max_brake, -ctrl.acceleration);
            a_prev = ctrl.acceleration;
            st = model->Step(st, ctrl, dt);
            t += dt; ++ticks;
            if (!std::isfinite(st.x) || !std::isfinite(st.y) || !std::isfinite(delta)) { outcome = sim::Outcome::OffMap; break; }

            int seg; float cte = SignedDistToPath(path, st.x, st.y, seg);
            max_seg = std::max(max_seg, seg);
            sum_cte2 += cte * cte; max_cte = std::max(max_cte, std::abs(cte));
            if (std::abs(delta) >= dmax - 1e-4f) ++sat_ticks;
            if (rebuilt && t < A.rebuild_at + 0.5f) dsteer_after_rebuild = std::max(dsteer_after_rebuild, std::abs(delta - prev_cmd_before_rebuild));
            steer_tv += std::abs(delta - prev_cmd); prev_cmd = delta;
            applied_tv += std::abs(st.steering_angle - prev_applied); prev_applied = st.steering_angle;
            float sg = (delta > 1e-3f) ? 1.f : (delta < -1e-3f ? -1.f : 0.f);
            if (sg != 0 && prev_sign != 0 && sg != prev_sign) ++sign_changes;
            if (sg != 0) prev_sign = sg;
            max_lat_acc = std::max(max_lat_acc, std::abs(st.speed * st.yaw_rate));
            max_speed_seen = std::max(max_speed_seen, st.speed);
            if (tr) std::fprintf(tr, "%.4f,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%.5f,%.5f,%.3f,%.4f\n",
                                 t, st.x, st.y, st.heading, st.speed, st.lateral_speed, st.yaw_rate, delta, st.steering_angle, ctrl.acceleration, cte);

            outcome = sim::CheckTermination(st, vd, map, path, path_len, t, maxSpeed);
        }
    }
    if (tr) std::fclose(tr);

    std::printf("{\"tag\":\"%s\",\"map\":\"%s\",\"vehicle\":\"%s\",\"algo\":\"%s\",\"fidelity\":\"%s\","
                "\"frame_dt\":%.5f,\"jitter\":[%.3f,%.3f],\"target_speed\":%.2f,\"speed_cap\":%.2f,"
                "\"outcome\":\"%s\",\"clean\":%s,\"t_end\":%.2f,\"ticks\":%ld,\"progress\":%.3f,"
                "\"rms_cte\":%.3f,\"max_cte\":%.3f,\"sat_frac\":%.3f,\"max_lat_acc\":%.2f,\"max_speed\":%.2f,"
                "\"max_accel\":%.2f,\"max_brake\":%.2f,\"max_jerk\":%.2f,"
                "\"steer_tv_per_s\":%.4f,\"applied_tv_per_s\":%.4f,\"sign_changes_per_s\":%.3f,"
                "\"ctrl_us_avg\":%.1f,\"ctrl_us_max\":%.1f,\"path_len_m\":%.1f,\"max_delta_deg\":%.2f,"
                "\"rebuild_at\":%.2f,\"dsteer_after_rebuild_deg\":%.2f,\"switch_at\":%.2f}\n",
                A.tag.c_str(), map.name.c_str(), vd.name.c_str(), A.algo.c_str(), A.fidelity.c_str(),
                A.frame_dt, A.jlo, A.jhi, maxSpeed, feas.speed_cap_mps,
                sim::OutcomeName(outcome), outcome == sim::Outcome::Goal ? "true" : "false", t, ticks,
                path.size() > 1 ? (float)(max_seg + 1) / (float)(path.size() - 1) : 0.f,
                ticks ? std::sqrt(sum_cte2 / static_cast<double>(ticks)) : 0.0, max_cte,
                ticks ? static_cast<double>(sat_ticks) / static_cast<double>(ticks) : 0.0,
                max_lat_acc, max_speed_seen, max_accel, max_brake, max_abs_jerk,
                t > 0 ? steer_tv / t : 0.0, t > 0 ? applied_tv / t : 0.0, t > 0 ? static_cast<double>(sign_changes) / t : 0.0,
                ticks ? ctrl_us_sum / static_cast<double>(ticks) : 0.0, ctrl_us_max, path_len, dmax * 57.2958f,
                A.rebuild_at, dsteer_after_rebuild * 57.2958f, A.switch_at);
    return 0;
}
