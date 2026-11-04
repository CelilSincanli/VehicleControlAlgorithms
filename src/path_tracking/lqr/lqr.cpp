#include "path_tracking/lqr/lqr.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace path_tracking {

namespace {

// ── Fixed-size matrix operations ──────────────────────────────────────────────
// All matrices use [row][col] indexing. Zero-initialize with = {}.

template <int R, int C>
Mat<C, R> transpose(const Mat<R, C>& a) {
    Mat<C, R> r = {};
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j)
            r[j][i] = a[i][j];
    return r;
}

template <int R, int K, int C>
Mat<R, C> mul(const Mat<R, K>& a, const Mat<K, C>& b) {
    Mat<R, C> r = {};
    for (int i = 0; i < R; ++i)
        for (int k = 0; k < K; ++k)
            for (int j = 0; j < C; ++j)
                r[i][j] += a[i][k] * b[k][j];
    return r;
}

template <int R, int C>
Mat<R, C> add(const Mat<R, C>& a, const Mat<R, C>& b) {
    Mat<R, C> r = a;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j)
            r[i][j] += b[i][j];
    return r;
}

template <int R, int C>
Mat<R, C> sub(const Mat<R, C>& a, const Mat<R, C>& b) {
    Mat<R, C> r = a;
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C; ++j)
            r[i][j] -= b[i][j];
    return r;
}

} // anonymous namespace

// ── Class methods ──────────────────────────────────────────────────────────────

Lqr::Lqr(const LqrConfig& config) : config_(config) {}

void Lqr::SetPath(const std::vector<Point2D>& waypoints) {
    path_             = waypoints;
    current_path_idx_ = 0;
    seeded_           = false;
    last_target_      = {};
    has_P_prev_       = false;
}

float Lqr::ComputeSteering(const vehicle::VehicleState& state) const {
    if (path_.size() < 3) return 0.0f;

    const int closest = FindClosestWaypoint(state);
    const int i       = std::min(closest, static_cast<int>(path_.size()) - 2);

    // Tracking errors against the segment (i, i+1)
    const float path_angle = std::atan2(path_[i + 1].y - path_[i].y, path_[i + 1].x - path_[i].x);
    const float dx = state.x - path_[i].x;
    const float dy = state.y - path_[i].y;
    const float e_lat = std::cos(path_angle) * dy - std::sin(path_angle) * dx;  // + = left of path
    const float e_yaw = NormalizeAngle(state.heading - path_angle);
    const float kappa = ComputeCurvature(closest);

    const float v  = std::max(state.speed, 0.1f);  // floor keeps B's steering column non-zero
    const float L  = config_.wheelbase;
    const float dt = config_.time_step;

    const float delta_ff = std::atan(L * kappa);
    Vec6 x = {
        e_lat,
        state.speed * std::sin(e_yaw),                 // de_lat
        e_yaw,
        state.yaw_rate - state.speed * kappa,          // de_yaw
        state.speed - config_.max_speed,               // e_v
        state.steering_angle - delta_ff,               // steering error vs feed-forward
    };

    Mat6x6 A = BuildA(v, dt, L);
    Mat6x2 B = BuildB(v, dt, L);

    Mat6x6 Q = {};
    Q[0][0] = config_.q0; Q[1][1] = config_.q1; Q[2][2] = config_.q2;
    Q[3][3] = config_.q3; Q[4][4] = config_.q4; Q[5][5] = config_.q5;

    Mat2x2 R = {};
    R[0][0] = config_.r_scale * config_.r_steering;
    R[1][1] = config_.r_scale * config_.r_acceleration;

    // Warm start from last tick's solution: speed changes little between
    // frames, so the iteration converges in a few steps instead of ~100.
    Mat6x6 P = SolveDARE(A, B, Q, R, config_.dare_iterations, config_.dare_threshold,
                         has_P_prev_ ? P_prev_ : Q);
    P_prev_     = P;
    has_P_prev_ = true;
    Mat2x6 K = ComputeGain(P, A, B, R);

    float rate = 0.0f;
    for (int j = 0; j < 6; ++j) rate -= K[0][j] * x[j];
    rate = std::max(-config_.max_delta_rate, std::min(config_.max_delta_rate, rate));

    const float cmd = std::max(-config_.max_delta,
                               std::min(config_.max_delta, state.steering_angle + rate * dt));

    last_target_   = path_[std::min(i + 5, static_cast<int>(path_.size()) - 1)];
    last_e_lat_    = e_lat;
    last_e_yaw_    = e_yaw;
    last_at_clamp_ = std::abs(cmd) >= config_.max_delta - 1e-6f;
    return cmd;
}

bool Lqr::GetLastTrackingDiagnostics(TrackingDiagnostics& out) const {
    out.cross_track_error_m = last_e_lat_;
    out.heading_error_deg   = last_e_yaw_ * (180.0f / 3.14159265f);
    out.nearest_idx         = current_path_idx_;
    out.last_cost           = 0.0f;
    out.steering_at_clamp   = last_at_clamp_;
    return true;
}

int Lqr::FindClosestWaypoint(const vehicle::VehicleState& state) const {
    const int n  = static_cast<int>(path_.size());
    // First call after SetPath(): search the whole path, so a rebuild mid-run
    // (live parameter change) picks up where the vehicle really is.
    int start    = seeded_ ? current_path_idx_ : 0;
    int end      = seeded_ ? std::min(n, start + config_.search_window) : n;
    int idx      = start;
    float min_d2 = std::numeric_limits<float>::max();
    for (int i = start; i < end; ++i) {
        float ddx = path_[i].x - state.x;
        float ddy = path_[i].y - state.y;
        float d2  = ddx * ddx + ddy * ddy;
        if (d2 < min_d2) { min_d2 = d2; idx = i; }
    }
    current_path_idx_ = idx;
    seeded_           = true;
    return idx;
}

float Lqr::ComputeCurvature(int idx) const {
    int i = std::max(0, idx - 1);
    int j = idx;
    int k = std::min((int)path_.size() - 1, idx + 1);
    if (i == j || j == k) return 0.0f;

    float ax = path_[j].x - path_[i].x, ay = path_[j].y - path_[i].y;
    float bx = path_[k].x - path_[i].x, by = path_[k].y - path_[i].y;
    float cross = ax * by - ay * bx;   // signed: + left turn, - right turn
    float la    = std::hypot(ax, ay);
    float lb    = std::hypot(bx - ax, by - ay);
    float lc    = std::hypot(bx, by);
    float denom = la * lb * lc;
    return (denom < 1e-9f) ? 0.0f : 2.0f * cross / denom;
}

Mat6x6 Lqr::BuildA(float v, float dt, float wheelbase) {
    Mat6x6 A = {};
    A[0][0] = 1.0f; A[0][1] = dt;                  // e_lat  += dt * de_lat
    A[1][2] = v;    A[1][3] = v * dt;              // de_lat  = v * e_yaw(next)
    A[2][2] = 1.0f; A[2][3] = dt;                  // e_yaw  += dt * de_yaw
    A[3][5] = v / wheelbase;                       // de_yaw  = v/L * (delta - delta_ff)
    A[4][4] = 1.0f;                                // e_v    += dt * accel
    A[5][5] = 1.0f;                                // steer  += dt * rate
    return A;
}

Mat6x2 Lqr::BuildB(float v, float dt, float wheelbase) {
    Mat6x2 B = {};
    B[3][0] = v / wheelbase * dt;
    B[4][1] = dt;
    B[5][0] = dt;
    return B;
}

bool Lqr::Invert2x2(const Mat2x2& m, Mat2x2& out) {
    float det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    if (std::abs(det) < 1e-12f) return false;
    float inv = 1.0f / det;
    out[0][0] =  m[1][1] * inv;
    out[0][1] = -m[0][1] * inv;
    out[1][0] = -m[1][0] * inv;
    out[1][1] =  m[0][0] * inv;
    return true;
}

float Lqr::NormalizeAngle(float a) {
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    while (a >  static_cast<float>(M_PI)) a -= two_pi;
    while (a < -static_cast<float>(M_PI)) a += two_pi;
    return a;
}

Mat6x6 Lqr::SolveDARE(const Mat6x6& A, const Mat6x2& B,
                       const Mat6x6& Q, const Mat2x2& R,
                       int iters, float thresh, const Mat6x6& P0) {
    Mat6x6 P  = P0;
    Mat6x6 At = transpose<6, 6>(A);
    Mat2x6 Bt = transpose<6, 2>(B);

    for (int iter = 0; iter < iters; ++iter) {
        Mat6x2 PB   = mul<6, 6, 2>(P, B);
        Mat2x2 S    = add<2, 2>(R, mul<2, 6, 2>(Bt, PB));
        Mat2x2 Sinv = {};
        if (!Invert2x2(S, Sinv)) break;

        Mat6x6 AtP      = mul<6, 6, 6>(At, P);
        Mat6x6 AtPA     = mul<6, 6, 6>(AtP, A);
        Mat6x2 AtPB     = mul<6, 6, 2>(AtP, B);
        Mat2x6 BtPA     = mul<2, 6, 6>(Bt, mul<6, 6, 6>(P, A));
        Mat6x6 Corr     = mul<6, 2, 6>(AtPB, mul<2, 2, 6>(Sinv, BtPA));
        Mat6x6 P_new    = sub<6, 6>(add<6, 6>(AtPA, Q), Corr);

        float maxDiff = 0.0f;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                maxDiff = std::max(maxDiff, std::abs(P_new[i][j] - P[i][j]));

        P = P_new;
        if (maxDiff < thresh) break;
    }
    return P;
}

Mat2x6 Lqr::ComputeGain(const Mat6x6& P, const Mat6x6& A,
                         const Mat6x2& B, const Mat2x2& R) {
    Mat2x6 Bt   = transpose<6, 2>(B);
    Mat2x2 S    = add<2, 2>(R, mul<2, 6, 2>(Bt, mul<6, 6, 2>(P, B)));
    Mat2x2 Sinv = {};
    if (!Invert2x2(S, Sinv)) return {};  // degenerate: return zero gain
    return mul<2, 2, 6>(Sinv, mul<2, 6, 6>(Bt, mul<6, 6, 6>(P, A)));
}

} // namespace path_tracking
