#pragma once
#include "map/map_data.hpp"
#include <vector>

namespace path_tracking {

struct VehicleState {
    float x       = 0.0f;
    float y       = 0.0f;
    float heading = 0.0f; // radians, 0 = east
    float speed   = 0.0f;
};

struct PurePursuitConfig {
    float lookahead_distance = 1.0f;
    float wheelbase          = 0.5f;
};

class PurePursuit {
public:
    explicit PurePursuit(const PurePursuitConfig& config = {});

    void  SetPath(const std::vector<Point2D>& waypoints);
    float ComputeSteering(const VehicleState& state) const;

    const PurePursuitConfig& GetConfig() const { return config_; }

private:
    PurePursuitConfig      config_;
    std::vector<Point2D>   path_;

    int      FindClosestWaypoint (const VehicleState& state) const;
    Point2D  FindLookaheadPoint  (const VehicleState& state, int closest_idx) const;
};

} // namespace path_tracking
