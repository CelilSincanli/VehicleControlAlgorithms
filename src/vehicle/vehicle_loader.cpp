#include "vehicle/vehicle_loader.hpp"
#include "common/json_value.hpp"
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace {
using JVal = json::Value;
float tire_deg_effective(const VehicleData& d) {
    return d.max_steering_wheel_angle / d.steering_wheel_to_tire_angle_ratio;
}
} // anonymous namespace

bool VehicleLoader::Load(const std::string& json_path) {
    vehicle_ = VehicleData{};
    warnings_.clear();
    error_.clear();
    JVal root;
    if (!json::ParseFile(json_path, root, error_)) return false;   // syntax error: reject the file
    if (root.type != JVal::Obj) { error_ = "top level is not a JSON object"; return false; }

    vehicle_.name = root.at("name").asString();

    vehicle_.mass = (float)root.at("mass").asFloat();
    vehicle_.a    = (float)root.at("a").asFloat();
    vehicle_.b    = (float)root.at("b").asFloat();
    vehicle_.CA   = (float)root.at("CA").asFloat();

    vehicle_.minimum_turning_radius             = (float)root.at("minimum_turning_radius").asFloat();
    vehicle_.max_steering_wheel_angle           = (float)root.at("max_steering_wheel_angle").asFloat();
    vehicle_.min_steering_wheel_angle           = (float)root.at("min_steering_wheel_angle").asFloat();
    vehicle_.max_steering_wheel_rate            = (float)root.at("max_steering_wheel_rate").asFloat();
    vehicle_.steering_wheel_to_tire_angle_ratio = (float)root.at("steering_wheel_to_tire_angle_ratio").asFloat();

    vehicle_.wheelbase      = (float)root.at("wheelbase").asFloat();
    vehicle_.wheel_radius   = (float)root.at("wheel_radius").asFloat();
    vehicle_.wheel_width    = (float)root.at("wheel_width").asFloat();
    vehicle_.wheel_tread    = (float)root.at("wheel_tread").asFloat();
    vehicle_.front_overhang = (float)root.at("front_overhang").asFloat();
    vehicle_.rear_overhang  = (float)root.at("rear_overhang").asFloat();
    vehicle_.left_overhang  = (float)root.at("left_overhang").asFloat();
    vehicle_.right_overhang = (float)root.at("right_overhang").asFloat();
    vehicle_.vehicle_height = (float)root.at("vehicle_height").asFloat();

    // New flat scalar. Guarded (unlike the block above): an old vehicle*.json
    // without "cog_height" leaves VehicleData::cog_height at its
    // member-initializer default (0.0f) rather than an unconditional
    // root.at("cog_height").asFloat() silently overwriting it for the wrong
    // reason (missing key vs. explicit zero are different things).
    if (root.has("cog_height"))
        vehicle_.cog_height = (float)root.at("cog_height").asFloat();

    // Same guarded pattern. Absent key => VehicleData::yaw_inertia stays at
    // its member-initializer default (0.0f), which ComposedVehicleModel's
    // constructor treats as "not provided" and falls back to mass*a*b.
    if (root.has("yaw_inertia"))
        vehicle_.yaw_inertia = (float)root.at("yaw_inertia").asFloat();

    // Subsystem parameter blocks — all optional nested objects. Absent in
    // vehicle01.json/vehicle02.json's original form, so both parsed exactly
    // as before (every *Params::enabled stays false) before these were added.
    if (root.has("tire")) {
        const JVal& tire = root.at("tire");
        vehicle_.tire.enabled = true;
        if (tire.has("cornering_stiffness_coeff"))
            vehicle_.tire.cornering_stiffness_coeff = (float)tire.at("cornering_stiffness_coeff").asFloat();
        if (tire.has("friction_coefficient"))
            vehicle_.tire.friction_coefficient = (float)tire.at("friction_coefficient").asFloat();
    }

    if (root.has("suspension")) {
        vehicle_.suspension.enabled = true;
        // No further fields yet — cog_height (above) is what feeds RigidSuspensionModel.
    }

    // Longitudinal comfort limits for the speed controller (optional; defaults
    // in VehicleData are generic passenger-car comfort values).
    if (root.has("max_acceleration")) vehicle_.max_acceleration = (float)root.at("max_acceleration").asFloat();
    if (root.has("max_deceleration")) vehicle_.max_deceleration = (float)root.at("max_deceleration").asFloat();
    if (root.has("max_jerk"))         vehicle_.max_jerk         = (float)root.at("max_jerk").asFloat();

    if (root.has("steering")) {
        const JVal& steering = root.at("steering");
        vehicle_.steering.enabled = true;
        if (steering.has("use_rate_limit"))
            vehicle_.steering.use_rate_limit = (steering.at("use_rate_limit").asFloat() != 0.0);
    }

    Validate(root.keys);
    vehicle_.loaded = true;
    return true;
}

void VehicleLoader::Validate(const std::vector<std::string>& present_keys) {
    auto present = [&](const char* k) {
        return std::find(present_keys.begin(), present_keys.end(), k) != present_keys.end();
    };
    auto note = [&](const char* key, const char* what, float v) {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "%s %s -> %.4g", key, what, v);
        warnings_.emplace_back(buf);
    };
    // Physical plausibility ranges. A missing required key arrives as 0 from
    // the parser and is clamped to the nearest allowed value like any other
    // out-of-range number, so the vehicle stays usable (no 0 ratio -> inf lock,
    // no 0 wheelbase -> NaN) and the user is told which field was wrong.
    auto req = [&](const char* key, float& v, float lo, float hi) {
        if (!present(key)) { v = std::max(lo, std::min(hi, v)); note(key, "missing", v); return; }
        if (!std::isfinite(v) || v < lo || v > hi) {
            v = std::isfinite(v) ? std::max(lo, std::min(hi, v)) : lo;
            note(key, "out of range", v);
        }
    };
    auto opt = [&](float& v, const char* key, float lo, float hi) {
        if (std::isfinite(v) && v >= lo && v <= hi) return;
        v = std::isfinite(v) ? std::max(lo, std::min(hi, v)) : lo;
        note(key, "out of range", v);
    };
    VehicleData& d = vehicle_;
    req("mass",                               d.mass,                               200.0f,  60000.0f);
    req("a",                                  d.a,                                  0.2f,    8.0f);
    req("b",                                  d.b,                                  0.2f,    8.0f);
    req("CA",                                 d.CA,                                 0.0f,    0.01f);
    req("minimum_turning_radius",             d.minimum_turning_radius,             1.0f,    30.0f);
    req("max_steering_wheel_angle",           d.max_steering_wheel_angle,           90.0f,   1440.0f);
    req("min_steering_wheel_angle",           d.min_steering_wheel_angle,           -1440.0f, -90.0f);
    req("max_steering_wheel_rate",            d.max_steering_wheel_rate,            30.0f,   1500.0f);
    req("steering_wheel_to_tire_angle_ratio", d.steering_wheel_to_tire_angle_ratio, 5.0f,    40.0f);
    req("wheelbase",                          d.wheelbase,                          1.0f,    12.0f);
    req("wheel_radius",                       d.wheel_radius,                       0.15f,   0.8f);
    req("wheel_width",                        d.wheel_width,                        0.1f,    0.6f);
    req("wheel_tread",                        d.wheel_tread,                        0.8f,    3.0f);
    req("front_overhang",                     d.front_overhang,                     0.0f,    5.0f);
    req("rear_overhang",                      d.rear_overhang,                      0.0f,    5.0f);
    req("left_overhang",                      d.left_overhang,                      0.0f,    1.0f);
    req("right_overhang",                     d.right_overhang,                     0.0f,    1.0f);
    req("vehicle_height",                     d.vehicle_height,                     0.8f,    5.0f);
    opt(d.max_acceleration, "max_acceleration", 0.3f, 6.0f);   // [m/s^2]
    opt(d.max_deceleration, "max_deceleration", 0.5f, 10.0f);  // [m/s^2]
    opt(d.max_jerk,         "max_jerk",         0.3f, 20.0f);  // [m/s^3]
    opt(d.cog_height,  "cog_height",  0.0f, 3.0f);        // 0 = load transfer disabled
    opt(d.yaw_inertia, "yaw_inertia", 0.0f, 1.0e6f);      // 0 = mass*a*b fallback
    if (d.tire.enabled) {
        opt(d.tire.cornering_stiffness_coeff, "tire.cornering_stiffness_coeff", 1.0f, 20.0f);
        opt(d.tire.friction_coefficient,      "tire.friction_coefficient",      0.1f, 1.5f);
    }
    // Derived max tire angle must be physical (10..60 deg); fix it through the
    // ratio, the field most likely to be missing or mistyped.
    const float tire_deg = d.max_steering_wheel_angle / d.steering_wheel_to_tire_angle_ratio;
    if (tire_deg > 60.0f || tire_deg < 10.0f) {
        d.steering_wheel_to_tire_angle_ratio = d.max_steering_wheel_angle / std::max(10.0f, std::min(60.0f, tire_deg));
        note("steering_wheel_to_tire_angle_ratio", "gives a tire angle outside 10-60 deg", d.steering_wheel_to_tire_angle_ratio);
    }
    // minimum_turning_radius is authoritative for the steering limit (see
    // vehicle::TireAngleLimits); flag a radius the steering geometry cannot reach.
    const float geom_radius = d.wheelbase / std::tan(tire_deg_effective(d) * 3.14159265f / 180.0f);
    if (d.minimum_turning_radius < geom_radius - 0.05f) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "minimum_turning_radius %.2f m is tighter than the steering allows (%.2f m); the steering limit applies",
                      d.minimum_turning_radius, geom_radius);
        warnings_.emplace_back(buf);
    }
    if (std::abs(d.a + d.b - d.wheelbase) > 0.05f) {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "a + b = %.3f m but wheelbase = %.3f m", d.a + d.b, d.wheelbase);
        warnings_.emplace_back(buf);
    }
}

std::vector<std::string> VehicleLoader::DiscoverVehicles(const std::string& vehicle_dir) {
    std::vector<std::string> result;
    namespace fs = std::filesystem;
    if (!fs::exists(vehicle_dir)) return result;

    for (const auto& entry : fs::directory_iterator(vehicle_dir)) {
        if (!entry.is_regular_file()) continue;
        if (entry.path().extension() == ".json")
            result.push_back(entry.path().string());
    }
    std::sort(result.begin(), result.end());
    return result;
}
