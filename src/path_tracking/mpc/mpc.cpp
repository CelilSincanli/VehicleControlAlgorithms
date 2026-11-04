#include "path_tracking/mpc/mpc.hpp"
#include "vehicle/vehicle_state.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace path_tracking {

namespace {

float SignedMenger(const Point2D& a, const Point2D& b, const Point2D& c) {
    float ab = std::hypot(b.x - a.x, b.y - a.y);
    float bc = std::hypot(c.x - b.x, c.y - b.y);
    float ca = std::hypot(a.x - c.x, a.y - c.y);
    float cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    float denom = ab * bc * ca;
    return (denom < 1e-9f) ? 0.0f : 2.0f * cross / denom;
}

float WrapAngle(float a) {
    const float pi = static_cast<float>(M_PI);
    while (a >  pi) a -= 2.0f * pi;
    while (a < -pi) a += 2.0f * pi;
    return a;
}

} // anonymous namespace

Mpc::Mpc(const MpcConfig& config) : config_(config) {}

void Mpc::SetPath(const std::vector<Point2D>& waypoints) {
    path_ = waypoints;
    current_path_idx_ = 0;
    seeded_ = false;
    target_point_ = path_.empty() ? Point2D{} : path_.front();

    const int n = static_cast<int>(path_.size());
    path_s_.assign(n, 0.0f);
    path_kappa_.assign(n, 0.0f);
    for (int i = 1; i < n; ++i)
        path_s_[i] = path_s_[i - 1] + std::hypot(path_[i].x - path_[i - 1].x, path_[i].y - path_[i - 1].y);
    for (int i = 2; i + 2 < n; ++i)
        path_kappa_[i] = SignedMenger(path_[i - 2], path_[i], path_[i + 2]);

    delta_inc_.assign(std::max(0, config_.horizon_N), 0.0f);
}

int Mpc::FindNearestWaypoint(float x, float y) const {
    const int n   = static_cast<int>(path_.size());
    const int beg = seeded_ ? current_path_idx_ : 0;               // first call: whole path
    const int end = seeded_ ? std::min(beg + config_.search_window, n) : n;
    float best    = std::numeric_limits<float>::max();
    int   result  = beg;
    for (int i = beg; i < end; ++i) {
        float dx = path_[i].x - x;
        float dy = path_[i].y - y;
        float d2 = dx * dx + dy * dy;
        if (d2 < best) { best = d2; result = i; }
    }
    seeded_ = true;
    return result;
}

float Mpc::CurvatureAt(float s) const {
    if (s <= path_s_.front()) return path_kappa_.front();
    if (s >= path_s_.back())  return 0.0f;   // past the end: straight
    auto it = std::upper_bound(path_s_.begin(), path_s_.end(), s);
    const int i = static_cast<int>(it - path_s_.begin()) - 1;
    const float t = (s - path_s_[i]) / std::max(1e-6f, path_s_[i + 1] - path_s_[i]);
    return path_kappa_[i] + t * (path_kappa_[i + 1] - path_kappa_[i]);
}

// Feasible set: |inc_k| <= max_step and |d0 + sum_{j<=k} inc_j| <= max_delta.
// Walking forward and clipping the running angle shrinks each increment toward
// zero, so the rate bound stays satisfied.
void Mpc::ProjectFeasible(std::vector<float>& inc, float d0, float max_step) const {
    float d = d0;
    for (float& v : inc) {
        v = std::max(-max_step, std::min(max_step, v));
        float nd = std::max(-config_.max_delta, std::min(config_.max_delta, d + v));
        v = nd - d;
        d = nd;
    }
}

float Mpc::ComputeSteering(const vehicle::VehicleState& state) const {
    const int N = config_.horizon_N;
    if (path_.size() < 5 || N <= 0 || static_cast<int>(delta_inc_.size()) != N) return 0.0f;

    const int nearest = FindNearestWaypoint(state.x, state.y);
    current_path_idx_ = nearest;
    target_point_     = path_[nearest];

    // Errors at the real pose, against the segment that starts at `nearest`
    const int   i  = std::min(nearest, static_cast<int>(path_.size()) - 2);
    const float pw = std::atan2(path_[i + 1].y - path_[i].y, path_[i + 1].x - path_[i].x);
    const float e_lat0 = -std::sin(pw) * (state.x - path_[i].x) + std::cos(pw) * (state.y - path_[i].y);
    const float e_psi0 = WrapAngle(state.heading - pw);
    last_e_lat_ = e_lat0;
    last_e_psi_ = e_psi0;

    const float L   = config_.wheelbase;
    const float dt  = config_.rollout_dt;
    const float v   = std::max(state.speed, 0.5f);
    const float d0  = state.steering_angle;
    const float max_step = (config_.max_delta_rate > 0.0f && std::isfinite(config_.max_delta_rate))
                               ? config_.max_delta_rate * dt : 2.0f * config_.max_delta;
    // arc length of the vehicle's projection onto the nearest segment
    const float along = path_s_[i] + std::cos(pw) * (state.x - path_[i].x) + std::sin(pw) * (state.y - path_[i].y);

    // Preview: feed-forward angle and linearised gain per step
    std::vector<float>& dff = dff_;  dff.assign(N, 0.0f);
    std::vector<float>& gk  = gain_; gk.assign(N, 0.0f);
    for (int k = 0; k < N; ++k) {
        float kap = CurvatureAt(along + v * dt * static_cast<float>(k));
        dff[k] = std::atan(L * kap);
        float c = std::cos(dff[k]);
        gk[k] = v / L / (c * c);
    }

    // Sensitivities of e_psi[k], e_lat[k] (k = 1..N) to the steering
    // deviations D_j = d_j - dff_j (closed form of the linear recursion).
    M_psi_.assign(static_cast<size_t>(N) * N, 0.0f);
    M_lat_.assign(static_cast<size_t>(N) * N, 0.0f);
    std::vector<float>& c_psi = c_psi_; c_psi.assign(N, 0.0f);
    std::vector<float>& c_lat = c_lat_; c_lat.assign(N, 0.0f);
    for (int k = 1; k <= N; ++k) {
        c_psi[k - 1] = e_psi0;
        c_lat[k - 1] = e_lat0 + static_cast<float>(k) * dt * v * e_psi0;
        for (int j = 0; j < k; ++j) {
            M_psi_[(k - 1) * N + j] = dt * gk[j];
            M_lat_[(k - 1) * N + j] = dt * dt * v * gk[j] * static_cast<float>(k - 1 - j);
        }
    }
    // D = T inc + d0vec,  T lower-triangular ones,  d0vec_k = d0 - dff_k
    // e = c + M D  ->  e = (c + M d0vec) + (M T) inc
    auto applyT = [&](const std::vector<float>& m, std::vector<float>& mt) {   // mt = m * T
        mt.assign(static_cast<size_t>(N) * N, 0.0f);
        for (int r = 0; r < N; ++r) {
            float acc = 0.0f;
            for (int c = N - 1; c >= 0; --c) { acc += m[r * N + c]; mt[r * N + c] = acc; }
        }
    };
    std::vector<float>& MTpsi = MT_psi_;
    std::vector<float>& MTlat = MT_lat_;
    applyT(M_psi_, MTpsi);
    applyT(M_lat_, MTlat);
    std::vector<float>& f_psi = f_psi_; f_psi.assign(N, 0.0f);
    std::vector<float>& f_lat = f_lat_; f_lat.assign(N, 0.0f);
    for (int r = 0; r < N; ++r) {
        float sp = c_psi[r], sl = c_lat[r];
        for (int c = 0; c < N; ++c) { sp += M_psi_[r * N + c] * (d0 - dff[c]); sl += M_lat_[r * N + c] * (d0 - dff[c]); }
        f_psi[r] = sp; f_lat[r] = sl;
    }

    // Quadratic form J = 0.5 inc' H inc + g' inc (+ const)
    const float wr = config_.w_rate / (dt * dt);
    H_.assign(static_cast<size_t>(N) * N, 0.0f);
    g_.assign(N, 0.0f);
    for (int a = 0; a < N; ++a) {
        for (int b = 0; b < N; ++b) {
            float h = 0.0f;
            for (int r = 0; r < N; ++r)
                h += config_.w_lat * MTlat[r * N + a] * MTlat[r * N + b] + config_.w_heading * MTpsi[r * N + a] * MTpsi[r * N + b];
            // steering deviation term: D = T inc + d0vec -> T'T entries = N - max(a,b)
            h += config_.w_control * static_cast<float>(N - std::max(a, b));
            if (a == b) h += wr;
            H_[a * N + b] = 2.0f * h;
        }
        float gg = 0.0f;
        for (int r = 0; r < N; ++r)
            gg += config_.w_lat * MTlat[r * N + a] * f_lat[r] + config_.w_heading * MTpsi[r * N + a] * f_psi[r];
        for (int k = a; k < N; ++k) gg += config_.w_control * (d0 - dff[k]);
        g_[a] = 2.0f * gg;
    }

    // FISTA (accelerated projected gradient), step 1/Lip with Lip from Gershgorin
    float lip = 1e-6f;
    for (int a = 0; a < N; ++a) {
        float row = 0.0f;
        for (int b = 0; b < N; ++b) row += std::abs(H_[a * N + b]);
        lip = std::max(lip, row);
    }
    const float step = 1.0f / lip;

    std::vector<float>& x = delta_inc_;      // warm start: previous solution
    ProjectFeasible(x, d0, max_step);
    std::vector<float>& xprev = x_prev_; xprev = x;
    std::vector<float>& z     = z_;      z     = x;
    grad_.assign(N, 0.0f);
    float t = 1.0f;
    for (int it = 0; it < std::max(1, config_.iterations); ++it) {
        for (int a = 0; a < N; ++a) {
            float gsum = g_[a];
            for (int b = 0; b < N; ++b) gsum += H_[a * N + b] * z[b];
            grad_[a] = gsum;
        }
        xprev = x;
        for (int a = 0; a < N; ++a) x[a] = z[a] - step * grad_[a];
        ProjectFeasible(x, d0, max_step);
        const float t_next = 0.5f * (1.0f + std::sqrt(1.0f + 4.0f * t * t));
        for (int a = 0; a < N; ++a) z[a] = x[a] + ((t - 1.0f) / t_next) * (x[a] - xprev[a]);
        t = t_next;
    }

    float cost = 0.0f;
    for (int a = 0; a < N; ++a) {
        float hx = 0.0f;
        for (int b = 0; b < N; ++b) hx += H_[a * N + b] * x[b];
        cost += 0.5f * x[a] * hx + g_[a] * x[a];
    }
    last_cost_ = cost;

    // Command for this control period: move along the first planned increment
    // at its planned rate (feasible for the actuator by construction).
    const float frac = std::min(1.0f, config_.control_dt / dt);
    float cmd = d0 + x[0] * frac;
    cmd = std::max(-config_.max_delta, std::min(config_.max_delta, cmd));
    last_steering_at_clamp_ = std::abs(cmd) >= config_.max_delta - 1e-6f;
    return cmd;
}

bool Mpc::GetLastTrackingDiagnostics(TrackingDiagnostics& out) const {
    out.cross_track_error_m = last_e_lat_;
    out.heading_error_deg   = last_e_psi_ * (180.0f / 3.14159265f);
    out.nearest_idx         = current_path_idx_;
    out.last_cost           = last_cost_;
    out.steering_at_clamp   = last_steering_at_clamp_;
    return true;
}

} // namespace path_tracking
