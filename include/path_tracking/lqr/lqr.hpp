#pragma once
#include "path_tracking/path_tracking_algorithm.hpp"
#include <array>
#include <limits>
#include <vector>

namespace path_tracking {

// Fixed-size matrices, [row][col]
template <int R, int C> using Mat = std::array<std::array<float, C>, R>;
using Mat6x6 = Mat<6, 6>;
using Mat6x2 = Mat<6, 2>;   // B
using Mat2x6 = Mat<2, 6>;   // Bᵀ, K
using Mat2x2 = Mat<2, 2>;
using Vec6   = std::array<float, 6>;

struct LqrConfig {
    // Q diagonal weights [lat_err, dlat_err, yaw_err, dyaw_err, speed_err, steer_err]
    float q0 = 1.0f, q1 = 1.0f, q2 = 1.0f, q3 = 1.0f, q4 = 1.0f, q5 = 1.0f;
    // R diagonal weights on [steering rate, acceleration] (each multiplied by r_scale)
    float r_steering = 1.0f, r_acceleration = 1.0f, r_scale = 4.0f;
    // Vehicle & timing
    float wheelbase = 0.5f;
    float time_step = 1.0f / 60.0f;  // control period used for A, B and for the output [s]
    float max_speed = 5.0f;          // target cruise speed [m/s]
    float max_delta = 1.0f;          // tire-angle limit [rad]
    float max_delta_rate = std::numeric_limits<float>::max();  // actuator rate limit [rad/s]
    // DARE solver
    int   dare_iterations = 150;
    float dare_threshold  = 0.01f;
    // Path search
    int   search_window   = 40;   // max waypoints to scan ahead for nearest point
};

// ---------------------------------------------------------------------------
// LQR with an actuator state
// ---------------------------------------------------------------------------
// State  x = [e_lat, de_lat, e_yaw, de_yaw, e_v, delta - delta_ff]
// Input  u = [steering rate, acceleration]
// The applied tire angle (VehicleState::steering_angle) is part of the state,
// so the gain accounts for the time the steering needs to move; the output is
// the applied angle advanced by one control period at the optimal rate.
// Error rates come from the vehicle state (v sin e_yaw, r - v kappa), not from
// finite differences, so the controller has no derivative kick after a
// rebuild and no dependence on the frame time.
// ---------------------------------------------------------------------------
class Lqr : public IPathTrackingAlgorithm {
public:
    explicit Lqr(const LqrConfig& config = {});

    void    SetPath(const std::vector<Point2D>& waypoints) override;
    float   ComputeSteering(const vehicle::VehicleState& state) const override;
    Point2D GetLookaheadPoint() const override { return last_target_; }
    void    SetActuatorRate(float max_delta_rate) override { config_.max_delta_rate = max_delta_rate; }
    bool    GetLastTrackingDiagnostics(TrackingDiagnostics& out) const override;

    const LqrConfig& GetConfig() const { return config_; }

private:
    LqrConfig            config_;
    std::vector<Point2D> path_;

    mutable int     current_path_idx_ = 0;
    mutable bool    seeded_           = false;  // first call searches the whole path
    mutable Point2D last_target_{};
    mutable Mat6x6  P_prev_{};                  // last DARE solution, warm start for the next tick
    mutable bool    has_P_prev_       = false;
    mutable float   last_e_lat_ = 0.0f, last_e_yaw_ = 0.0f;
    mutable bool    last_at_clamp_ = false;

    int   FindClosestWaypoint(const vehicle::VehicleState& state) const;
    float ComputeCurvature(int idx) const;   // signed: + left turn, - right turn

    static Mat6x6 BuildA(float v, float dt, float wheelbase);
    static Mat6x2 BuildB(float v, float dt, float wheelbase);
    static Mat6x6 SolveDARE(const Mat6x6& A, const Mat6x2& B,
                             const Mat6x6& Q, const Mat2x2& R,
                             int iters, float thresh, const Mat6x6& P0);
    static Mat2x6 ComputeGain(const Mat6x6& P, const Mat6x6& A,
                               const Mat6x2& B, const Mat2x2& R);
    static bool   Invert2x2(const Mat2x2& m, Mat2x2& out);
    static float  NormalizeAngle(float a);
};

} // namespace path_tracking
