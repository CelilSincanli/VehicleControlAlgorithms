#include "path_tracking/path_feasibility.hpp"
#include "vehicle/vehicle_model_factory.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace path_tracking {

namespace {

constexpr int   kCurvatureWindow       = 2;      // +-waypoints for the Menger estimate
constexpr float kLateralFrictionShare  = 0.5f;   // use half the friction circle for cornering
constexpr float kSteerRateShare        = 0.65f;  // leave 35 % of the actuator rate for feedback corrections
constexpr float kMinSpeedCap           = 1.0f;   // [m/s]
constexpr float kMaxSpeedCap           = 15.0f;  // [m/s]
constexpr float kGravity               = 9.81f;

float SignedMenger(const Point2D& a, const Point2D& b, const Point2D& c) {
    float ab = std::hypot(b.x - a.x, b.y - a.y);
    float bc = std::hypot(c.x - b.x, c.y - b.y);
    float ca = std::hypot(a.x - c.x, a.y - c.y);
    float cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    float denom = ab * bc * ca;
    return (denom < 1e-9f) ? 0.0f : 2.0f * cross / denom;
}

} // anonymous namespace

PathFeasibility AnalyzePathFeasibility(const std::vector<Point2D>& path, const VehicleData& vd) {
    PathFeasibility out;
    const float max_tire = vehicle::MaxTireAngle(vd);
    out.vehicle_min_radius_m = (max_tire > 1e-6f) ? vd.wheelbase / std::tan(max_tire)
                                                  : std::numeric_limits<float>::infinity();
    const int n = static_cast<int>(path.size());
    const int w = kCurvatureWindow;
    if (n < 2 * w + 3 || vd.wheelbase <= 0.0f) {
        out.v_lateral_mps = out.v_steer_rate_mps = out.speed_cap_mps = kMaxSpeedCap;
        return out;
    }

    // signed curvature, tire angle and arc length at every interior waypoint
    std::vector<float> delta(n, 0.0f), s(n, 0.0f);
    float kappa_max = 0.0f;
    for (int i = 1; i < n; ++i)
        s[i] = s[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    for (int i = w; i < n - w; ++i) {
        float k  = SignedMenger(path[i - w], path[i], path[i + w]);
        kappa_max = std::max(kappa_max, std::abs(k));
        delta[i]  = std::atan(vd.wheelbase * k);
    }

    float ddelta_ds_max = 0.0f;
    for (int i = 2 * w; i < n - 2 * w; ++i) {
        float ds = s[i + w] - s[i - w];
        if (ds > 1e-6f)
            ddelta_ds_max = std::max(ddelta_ds_max, std::abs(delta[i + w] - delta[i - w]) / ds);
    }

    out.path_min_radius_m      = (kappa_max > 1e-9f) ? 1.0f / kappa_max : std::numeric_limits<float>::infinity();
    out.steer_needed_rad       = std::atan(vd.wheelbase * kappa_max);
    out.geometrically_feasible = out.path_min_radius_m >= out.vehicle_min_radius_m;

    const float mu = vd.tire.enabled ? vd.tire.friction_coefficient : 1.0f;
    out.v_lateral_mps = (kappa_max > 1e-9f) ? std::sqrt(kLateralFrictionShare * mu * kGravity / kappa_max)
                                            : kMaxSpeedCap;

    const bool rate_limited = vd.steering.enabled && vd.steering.use_rate_limit && vd.max_steering_wheel_rate > 0.0f;
    if (rate_limited && ddelta_ds_max > 1e-9f) {
        float rate = vehicle::ToRadians(vd.max_steering_wheel_rate) / vd.steering_wheel_to_tire_angle_ratio;
        out.v_steer_rate_mps = kSteerRateShare * rate / ddelta_ds_max;
    } else {
        out.v_steer_rate_mps = std::numeric_limits<float>::infinity();
    }

    out.speed_cap_mps = std::max(kMinSpeedCap, std::min({kMaxSpeedCap, out.v_lateral_mps, out.v_steer_rate_mps}));
    return out;
}

} // namespace path_tracking
