#pragma once
#include "map/map_data.hpp"
#include "vehicle/vehicle_state.hpp"
#include <vector>

namespace path_tracking {

class IPathTrackingAlgorithm {
public:
    virtual ~IPathTrackingAlgorithm() = default;
    virtual void    SetPath(const std::vector<Point2D>& waypoints) = 0;
    virtual float   ComputeSteering(const vehicle::VehicleState& state) const = 0;
    virtual Point2D GetLookaheadPoint() const = 0;
};

} // namespace path_tracking
