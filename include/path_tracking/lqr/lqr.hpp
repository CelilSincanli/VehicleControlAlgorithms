#pragma once
#include "path_tracking/path_tracking_algorithm.hpp"
#include <array>
#include <vector>

namespace path_tracking {

using Mat5x5 = std::array<std::array<float, 5>, 5>;  // [row][col]
using Mat5x2 = std::array<std::array<float, 2>, 5>;  // B: 5 rows, 2 cols
using Mat2x5 = std::array<std::array<float, 5>, 2>;  // Bᵀ, K: 2 rows, 5 cols
using Mat2x2 = std::array<std::array<float, 2>, 2>;
using Vec5   = std::array<float, 5>;

struct LqrConfig {
    // Q diagonal weights [lat_err, dlat_err, yaw_err, dyaw_err, speed_err]
    float q0 = 1.0f, q1 = 1.0f, q2 = 1.0f, q3 = 1.0f, q4 = 1.0f;
    // R diagonal weights (each multiplied by r_scale)
    float r_steering = 1.0f, r_acceleration = 1.0f, r_scale = 4.0f;
    // Vehicle & timing
    float wheelbase = 0.5f;
    float time_step = 0.016f;   // fixed dt for system matrices A, B [s]
    float max_speed = 5.0f;     // target cruise speed [m/s]
    // DARE solver
    int   dare_iterations = 150;
    float dare_threshold  = 0.01f;
};

class Lqr : public IPathTrackingAlgorithm {
public:
    explicit Lqr(const LqrConfig& config = {});

    void    SetPath(const std::vector<Point2D>& waypoints) override;
    float   ComputeSteering(const vehicle::VehicleState& state) const override;
    Point2D GetLookaheadPoint() const override { return last_target_; }

    const LqrConfig& GetConfig() const { return config_; }

private:
    LqrConfig            config_;
    std::vector<Point2D> path_;

    mutable int     current_path_idx_ = 0;
    mutable Point2D last_target_{};
    mutable float   prev_lat_error_   = 0.0f;
    mutable float   prev_yaw_error_   = 0.0f;

    int   FindClosestWaypoint(const vehicle::VehicleState& state) const;
    float ComputeCurvature(int idx) const;
    Vec5  ComputeStateVector(const vehicle::VehicleState& state, int idx) const;

    static Mat5x5 BuildA(float v, float dt);
    static Mat5x2 BuildB(float v, float dt, float wheelbase);
    static Mat5x5 SolveDARE(const Mat5x5& A, const Mat5x2& B,
                             const Mat5x5& Q, const Mat2x2& R,
                             int iters, float thresh);
    static Mat2x5 ComputeGain(const Mat5x5& P, const Mat5x5& A,
                               const Mat5x2& B, const Mat2x2& R);
    static bool   Invert2x2(const Mat2x2& m, Mat2x2& out);
    static float  NormalizeAngle(float a);
};

} // namespace path_tracking
