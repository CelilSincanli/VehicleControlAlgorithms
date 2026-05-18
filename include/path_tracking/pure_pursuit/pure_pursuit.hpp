#pragma once
#include "map/map_data.hpp"
#include "vehicle/vehicle_state.hpp"
#include <vector>

namespace path_tracking {

struct PurePursuitConfig {
    float lookahead_distance = 2.0f;  // minimum lookahead [m]
    float lookahead_gain     = 0.3f;  // Ld = lookahead_gain * |v| + lookahead_distance
    float wheelbase          = 0.5f;
};

class PurePursuit {
public:
    explicit PurePursuit(const PurePursuitConfig& config = {});

    void  SetPath(const std::vector<Point2D>& waypoints);

    // Returns front-wheel steering angle δ [rad] given current vehicle state.
    float ComputeSteering(const vehicle::VehicleState& state) const;

    const PurePursuitConfig& GetConfig()       const { return config_; }
    Point2D                  GetLookaheadPoint() const { return last_lookahead_; }

private:
    PurePursuitConfig    config_;
    std::vector<Point2D> path_;
    mutable Point2D      last_lookahead_{};
    mutable int          current_path_idx_ = 0;

    static constexpr int kSearchWindow = 40;  // max waypoints to scan ahead

    int     FindClosestWaypoint(const vehicle::VehicleState& state) const;
    Point2D FindLookaheadPoint (const vehicle::VehicleState& state, int closest_idx, float ld) const;
};

} // namespace path_tracking
