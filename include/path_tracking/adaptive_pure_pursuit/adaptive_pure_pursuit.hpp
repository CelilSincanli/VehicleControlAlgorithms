#pragma once
#include "path_tracking/path_tracking_algorithm.hpp"
#include <vector>

namespace path_tracking {

struct AdaptivePurePursuitConfig {
    float min_lookahead  = 1.5f;  // [m] floor for Ld
    float max_lookahead  = 8.0f;  // [m] ceiling for Ld
    float speed_gain     = 0.5f;  // Ld grows with speed
    float curvature_gain = 3.0f;  // Ld shrinks with path curvature
    float wheelbase      = 0.5f;
    int   search_window  = 40;    // max waypoints to scan ahead for nearest point
};

class AdaptivePurePursuit : public IPathTrackingAlgorithm {
public:
    explicit AdaptivePurePursuit(const AdaptivePurePursuitConfig& config = {});

    void    SetPath(const std::vector<Point2D>& waypoints) override;
    float   ComputeSteering(const vehicle::VehicleState& state) const override;
    Point2D GetLookaheadPoint() const override { return last_lookahead_; }

    const AdaptivePurePursuitConfig& GetConfig() const { return config_; }

private:
    AdaptivePurePursuitConfig config_;
    std::vector<Point2D>      path_;
    mutable Point2D           last_lookahead_{};
    mutable int               current_path_idx_ = 0;

    int     FindClosestWaypoint(const vehicle::VehicleState& state) const;
    Point2D FindLookaheadPoint (const vehicle::VehicleState& state, int closest_idx, float ld) const;
    float   ComputeCurvature   (int waypoint_idx) const;
};

} // namespace path_tracking
