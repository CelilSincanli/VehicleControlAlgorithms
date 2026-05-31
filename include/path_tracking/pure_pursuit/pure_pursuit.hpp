#pragma once
#include "path_tracking/path_tracking_algorithm.hpp"
#include <vector>

namespace path_tracking {

struct PurePursuitConfig {
    float lookahead_distance = 2.0f;  // minimum lookahead [m]
    float lookahead_gain     = 0.3f;  // Ld = lookahead_gain * |v| + lookahead_distance
    float wheelbase          = 0.5f;
    int   search_window      = 40;    // max waypoints to scan ahead for nearest point
};

class PurePursuit : public IPathTrackingAlgorithm {
public:
    explicit PurePursuit(const PurePursuitConfig& config = {});

    void    SetPath(const std::vector<Point2D>& waypoints) override;
    float   ComputeSteering(const vehicle::VehicleState& state) const override;
    Point2D GetLookaheadPoint() const override { return last_lookahead_; }

    const PurePursuitConfig& GetConfig() const { return config_; }

private:
    PurePursuitConfig    config_;
    std::vector<Point2D> path_;
    mutable Point2D      last_lookahead_{};
    mutable int          current_path_idx_ = 0;

    int     FindClosestWaypoint(const vehicle::VehicleState& state) const;
    Point2D FindLookaheadPoint (const vehicle::VehicleState& state, int closest_idx, float ld) const;
};

} // namespace path_tracking
