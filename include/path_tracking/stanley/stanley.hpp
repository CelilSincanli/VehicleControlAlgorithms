#pragma once
#include "path_tracking/path_tracking_algorithm.hpp"
#include <vector>

namespace path_tracking {

struct StanleyConfig {
    float stanley_gain = 0.5f;  // k — cross-track error gain
    float wheelbase    = 4.0f;  // L [m]  — set from VehicleData at startup
    float min_speed    = 0.1f;  // v_min — divide-by-zero guard [m/s]
    float max_speed    = 5.0f;
};

class Stanley : public IPathTrackingAlgorithm {
public:
    explicit Stanley(const StanleyConfig& config = {});

    void    SetPath(const std::vector<Point2D>& waypoints) override;
    float   ComputeSteering(const vehicle::VehicleState& state) const override;
    Point2D GetLookaheadPoint() const override { return target_point_; }

    const StanleyConfig& GetConfig() const { return config_; }

private:
    StanleyConfig        config_;
    std::vector<Point2D> path_;
    mutable Point2D      target_point_{};
    mutable int          current_path_idx_ = 0;

    static constexpr int kSearchWindow = 40;

    int   FindNearestWaypoint(float fx, float fy) const;
    float ComputePathYaw(int index) const;
};

} // namespace path_tracking
