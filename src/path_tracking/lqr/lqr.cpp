#include "path_tracking/lqr/lqr.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace path_tracking {

namespace {

// ── Fixed-size matrix operations ──────────────────────────────────────────────
// All matrices use [row][col] indexing. Zero-initialize with = {}.

Mat5x5 transpose55(const Mat5x5& a) {
    Mat5x5 r = {};
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            r[i][j] = a[j][i];
    return r;
}

// B is Mat5x2 (5 rows, 2 cols); its transpose is Mat2x5 (2 rows, 5 cols)
Mat2x5 transpose52(const Mat5x2& a) {
    Mat2x5 r = {};
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 2; ++j)
            r[j][i] = a[i][j];
    return r;
}

// (5x5) × (5x5) → (5x5)
Mat5x5 mul55_55(const Mat5x5& a, const Mat5x5& b) {
    Mat5x5 r = {};
    for (int i = 0; i < 5; ++i)
        for (int k = 0; k < 5; ++k)
            for (int j = 0; j < 5; ++j)
                r[i][j] += a[i][k] * b[k][j];
    return r;
}

// (5x5) × (5x2) → (5x2)
Mat5x2 mul55_52(const Mat5x5& a, const Mat5x2& b) {
    Mat5x2 r = {};
    for (int i = 0; i < 5; ++i)
        for (int k = 0; k < 5; ++k)
            for (int j = 0; j < 2; ++j)
                r[i][j] += a[i][k] * b[k][j];
    return r;
}

// (2x5) × (5x2) → (2x2)
Mat2x2 mul25_52(const Mat2x5& a, const Mat5x2& b) {
    Mat2x2 r = {};
    for (int i = 0; i < 2; ++i)
        for (int k = 0; k < 5; ++k)
            for (int j = 0; j < 2; ++j)
                r[i][j] += a[i][k] * b[k][j];
    return r;
}

// (2x5) × (5x5) → (2x5)
Mat2x5 mul25_55(const Mat2x5& a, const Mat5x5& b) {
    Mat2x5 r = {};
    for (int i = 0; i < 2; ++i)
        for (int k = 0; k < 5; ++k)
            for (int j = 0; j < 5; ++j)
                r[i][j] += a[i][k] * b[k][j];
    return r;
}

// (2x2) × (2x5) → (2x5)
Mat2x5 mul22_25(const Mat2x2& a, const Mat2x5& b) {
    Mat2x5 r = {};
    for (int i = 0; i < 2; ++i)
        for (int k = 0; k < 2; ++k)
            for (int j = 0; j < 5; ++j)
                r[i][j] += a[i][k] * b[k][j];
    return r;
}

// (5x2) × (2x5) → (5x5)
Mat5x5 mul52_25(const Mat5x2& a, const Mat2x5& b) {
    Mat5x5 r = {};
    for (int i = 0; i < 5; ++i)
        for (int k = 0; k < 2; ++k)
            for (int j = 0; j < 5; ++j)
                r[i][j] += a[i][k] * b[k][j];
    return r;
}

Mat5x5 add55(const Mat5x5& a, const Mat5x5& b) {
    Mat5x5 r = a;
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            r[i][j] += b[i][j];
    return r;
}

Mat5x5 sub55(const Mat5x5& a, const Mat5x5& b) {
    Mat5x5 r = a;
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            r[i][j] -= b[i][j];
    return r;
}

Mat2x2 add22(const Mat2x2& a, const Mat2x2& b) {
    Mat2x2 r = a;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            r[i][j] += b[i][j];
    return r;
}

} // anonymous namespace

// ── Class methods ──────────────────────────────────────────────────────────────

Lqr::Lqr(const LqrConfig& config) : config_(config) {}

void Lqr::SetPath(const std::vector<Point2D>& waypoints) {
    path_             = waypoints;
    current_path_idx_ = 0;
    last_target_      = {};
    prev_lat_error_   = 0.0f;
    prev_yaw_error_   = 0.0f;
}

float Lqr::ComputeSteering(const vehicle::VehicleState& state) const {
    if (path_.size() < 2) return 0.0f;

    int closest = FindClosestWaypoint(state);

    // System matrices linearised at current speed; floor prevents zero B column
    float v  = std::max(state.speed, 0.1f);
    float dt = config_.time_step;
    Mat5x5 A = BuildA(v, dt);
    Mat5x2 B = BuildB(v, dt, config_.wheelbase);

    Mat5x5 Q = {};
    Q[0][0] = config_.q0; Q[1][1] = config_.q1; Q[2][2] = config_.q2;
    Q[3][3] = config_.q3; Q[4][4] = config_.q4;

    Mat2x2 R = {};
    R[0][0] = config_.r_scale * config_.r_steering;
    R[1][1] = config_.r_scale * config_.r_acceleration;

    Mat5x5 P = SolveDARE(A, B, Q, R, config_.dare_iterations, config_.dare_threshold);
    Mat2x5 K = ComputeGain(P, A, B, R);

    // ComputeStateVector also updates prev errors and last_target_
    Vec5 sv = ComputeStateVector(state, closest);

    float ff = std::atan2(config_.wheelbase * ComputeCurvature(closest), 1.0f);
    float fb = 0.0f;
    for (int j = 0; j < 5; ++j) fb += K[0][j] * sv[j];

    return NormalizeAngle(ff - fb);
}

int Lqr::FindClosestWaypoint(const vehicle::VehicleState& state) const {
    int start    = current_path_idx_;
    int end      = std::min((int)path_.size(), start + 40);
    int idx      = start;
    float min_d2 = std::numeric_limits<float>::max();
    for (int i = start; i < end; ++i) {
        float dx = path_[i].x - state.x;
        float dy = path_[i].y - state.y;
        float d2 = dx * dx + dy * dy;
        if (d2 < min_d2) { min_d2 = d2; idx = i; }
    }
    current_path_idx_ = idx;
    return idx;
}

float Lqr::ComputeCurvature(int idx) const {
    int i = std::max(0, idx - 1);
    int j = idx;
    int k = std::min((int)path_.size() - 1, idx + 1);
    if (i == j || j == k) return 0.0f;

    float ax = path_[j].x - path_[i].x, ay = path_[j].y - path_[i].y;
    float bx = path_[k].x - path_[i].x, by = path_[k].y - path_[i].y;
    float cross = std::abs(ax * by - ay * bx);
    float la    = std::hypot(ax, ay);
    float lb    = std::hypot(bx - ax, by - ay);
    float lc    = std::hypot(bx, by);
    float denom = la * lb * lc;
    return (denom < 1e-9f) ? 0.0f : 2.0f * cross / denom;
}

Vec5 Lqr::ComputeStateVector(const vehicle::VehicleState& state, int idx) const {
    int i = std::min(idx, (int)path_.size() - 2);

    float path_angle = std::atan2(path_[i+1].y - path_[i].y,
                                  path_[i+1].x - path_[i].x);
    float dx = state.x - path_[i].x;
    float dy = state.y - path_[i].y;

    // Signed lateral distance: positive = vehicle is left of path direction
    float lat_error = std::cos(path_angle) * dy - std::sin(path_angle) * dx;
    float yaw_error = NormalizeAngle(state.heading - path_angle);

    float dlat = (lat_error - prev_lat_error_) / config_.time_step;
    float dyaw = (yaw_error - prev_yaw_error_) / config_.time_step;

    prev_lat_error_ = lat_error;
    prev_yaw_error_ = yaw_error;
    last_target_    = path_[std::min(i + 5, (int)path_.size() - 1)];

    return {lat_error, dlat, yaw_error, dyaw, state.speed - config_.max_speed};
}

Mat5x5 Lqr::BuildA(float v, float dt) {
    Mat5x5 A = {};
    A[0][0] = 1.0f;
    A[0][1] = dt;
    A[1][2] = v;
    A[2][2] = 1.0f;
    A[2][3] = dt;
    A[4][4] = 1.0f;
    return A;
}

Mat5x2 Lqr::BuildB(float v, float dt, float wheelbase) {
    Mat5x2 B = {};
    B[3][0] = v / wheelbase;
    B[4][1] = dt;
    return B;
}

bool Lqr::Invert2x2(const Mat2x2& m, Mat2x2& out) {
    float det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    if (std::abs(det) < 1e-9f) return false;
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

Mat5x5 Lqr::SolveDARE(const Mat5x5& A, const Mat5x2& B,
                       const Mat5x5& Q, const Mat2x2& R,
                       int iters, float thresh) {
    Mat5x5 P  = Q;
    Mat5x5 At = transpose55(A);
    Mat2x5 Bt = transpose52(B);

    for (int iter = 0; iter < iters; ++iter) {
        Mat5x2 PB    = mul55_52(P, B);
        Mat2x2 BtPB  = mul25_52(Bt, PB);
        Mat2x2 S     = add22(R, BtPB);
        Mat2x2 Sinv  = {};
        if (!Invert2x2(S, Sinv)) break;

        Mat5x5 AtP     = mul55_55(At, P);
        Mat5x5 AtPA    = mul55_55(AtP, A);
        Mat5x2 AtPB    = mul55_52(AtP, B);
        Mat5x5 PA      = mul55_55(P, A);
        Mat2x5 BtPA    = mul25_55(Bt, PA);
        Mat2x5 SinvBtPA = mul22_25(Sinv, BtPA);
        Mat5x5 Corr    = mul52_25(AtPB, SinvBtPA);

        Mat5x5 P_new = sub55(add55(AtPA, Q), Corr);

        float maxDiff = 0.0f;
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 5; ++j)
                maxDiff = std::max(maxDiff, std::abs(P_new[i][j] - P[i][j]));

        P = P_new;
        if (maxDiff < thresh) break;
    }
    return P;
}

Mat2x5 Lqr::ComputeGain(const Mat5x5& P, const Mat5x5& A,
                         const Mat5x2& B, const Mat2x2& R) {
    Mat2x5 Bt   = transpose52(B);
    Mat5x2 PB   = mul55_52(P, B);
    Mat2x2 BtPB = mul25_52(Bt, PB);
    Mat2x2 S    = add22(R, BtPB);
    Mat2x2 Sinv = {};
    if (!Invert2x2(S, Sinv)) return {};  // degenerate: return zero gain

    Mat5x5 PA   = mul55_55(P, A);
    Mat2x5 BtPA = mul25_55(Bt, PA);
    return mul22_25(Sinv, BtPA);
}

} // namespace path_tracking
