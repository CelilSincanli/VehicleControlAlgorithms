#pragma once
#include "map/map_data.hpp"
#include "vehicle/vehicle_data.hpp"
#include "vehicle/vehicle_state.hpp"
#include <array>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Simulation rules shared by the GUI loop (VehicleControlUI) and the headless
// test harness (tests/sim_harness.cpp), so both run exactly the same logic.
// Units: metres, seconds, radians. Maps are authored in metres.
// ---------------------------------------------------------------------------
namespace sim {

constexpr float kControlDt = 1.0f / 60.0f;  // fixed control/physics step of the loop [s]

// --- Speed controller -------------------------------------------------------
// P control on speed error, limited to the vehicle's acceleration / braking
// limits and its jerk limit (relative to the previous command).
float SpeedCommand(float v_target, float v, float a_prev, const VehicleData& vd, float dt);

// --- Vehicle footprint ------------------------------------------------------
// Body rectangle around the rear-axle state (x, y): front = wheelbase +
// front_overhang, rear = rear_overhang, half width = tread/2 + side overhang.
using Quad = std::array<Point2D, 4>;
Quad BodyCorners(const vehicle::VehicleState& s, const VehicleData& vd);
bool BodyHitsObstacle(const Quad& body, const MapData& map);
bool BodyOffMap(const Quad& body, const MapData& map);

// --- Termination ------------------------------------------------------------
enum class Outcome { Running, Goal, Collision, OffMap, Timeout };
const char* OutcomeName(Outcome o);

struct TerminationRules {
    float goal_radius      = 1.0f;   // [m] rear axle this close to the goal = done
    float goal_lateral_tol = 3.0f;   // [m] or crossing the goal line within this distance of the path ...
    float goal_capture     = 10.0f;  // [m] ... and within this distance of the goal
    float timeout_factor   = 3.0f;   // timeout = factor * path_length / v_target + slack
    float timeout_slack    = 30.0f;  // [s]
};

float PathLength(const std::vector<Point2D>& path);

// Evaluated after every step. `elapsed` is simulation time, `v_target` the
// current target speed (the timeout follows live speed changes).
Outcome CheckTermination(const vehicle::VehicleState& s, const VehicleData& vd, const MapData& map,
                         const std::vector<Point2D>& path, float path_length,
                         float elapsed, float v_target, const TerminationRules& rules = {});

} // namespace sim
