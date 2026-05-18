#pragma once
#include "map/map_data.hpp"
#include "vehicle/vehicle_state.hpp"
#include <vector>

namespace path_tracking {

struct PurePursuitConfig {
    float lookahead_distance = 1.0f;
    float wheelbase          = 0.5f;
};

class PurePursuit {
public:
    explicit PurePursuit(const PurePursuitConfig& config = {});

    void  SetPath(const std::vector<Point2D>& waypoints);

    // Returns front-wheel steering angle δ [rad] given current vehicle state.
    float ComputeSteering(const vehicle::VehicleState& state) const;

    const PurePursuitConfig& GetConfig() const { return config_; }

private:
    PurePursuitConfig    config_;
    std::vector<Point2D> path_;

    int     FindClosestWaypoint(const vehicle::VehicleState& state) const;
    Point2D FindLookaheadPoint (const vehicle::VehicleState& state, int closest_idx) const;
};

} // namespace path_tracking
