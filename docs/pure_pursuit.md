# Pure Pursuit Path Tracking

This document covers the geometric foundation of the Pure Pursuit algorithm, how it is implemented in this project, and how its parameters affect tracking behaviour.

---

## Concept

Pure Pursuit is a **geometric path-tracking controller**. It picks a point on the reference path at a fixed look-ahead distance ahead of the vehicle, then computes the constant-curvature arc that the vehicle must follow to reach that point. The required steering angle is derived from that arc.

The algorithm is:
- Model-independent (no dynamic model inversion required)
- Computationally lightweight (one look-up + one `atan2`)
- Inherently stable — cross-track error drives steering which reduces cross-track error

---

## Geometric Derivation

Consider the vehicle's rear axle at position `P` with heading `ψ`. The look-ahead point is `L` at distance `Ld` from `P`.

```
                 L (look-ahead point)
                /|
               / |
           Ld /  | e  (lateral offset in vehicle frame)
             /   |
            /    |
           P ────┼──────► (vehicle heading ψ)
           α = look-ahead angle
```

The angle `α` between the vehicle heading and the line `PL` satisfies:

```
sin(α) = e / Ld
```

where `e` is the lateral component of `L` in the vehicle body frame (`local_y` in the code).

For a bicycle model of wheelbase `L`, the relationship between front-wheel steering angle `δ` and turning radius `R` is:

```
tan(δ) = L / R
```

Pure Pursuit chooses the radius `R` such that the circular arc from `P` passes through `L`. Elementary geometry gives:

```
R = Ld² / (2 · e)
```

Substituting:

```
tan(δ) = L / R = 2 · L · e / Ld²
```

And the final formula used in the code:

```
δ = atan2(2 · L · e, Ld²)
```

---

## Dynamic Look-Ahead Distance

A fixed look-ahead would be too aggressive at high speed and too slow to correct at low speed. The implementation uses a **speed-dependent look-ahead**:

```
Ld = lookahead_distance + lookahead_gain × |vx|
```

| Symbol | Config key | Default | Unit |
|--------|-----------|---------|------|
| `lookahead_distance` | `lookahead_distance` | 5.0 | m |
| `lookahead_gain` | `lookahead_gain` | 0.5 | — |
| `vx` | — | vehicle speed | m/s |

At the default cruise speed of 5 m/s:

```
Ld = 5.0 + 0.5 × 5.0 = 7.5 m
```

---

## Implementation

### Files

| File | Role |
|------|------|
| [include/path_tracking/pure_pursuit/pure_pursuit.hpp](../include/path_tracking/pure_pursuit/pure_pursuit.hpp) | Class interface and config struct |
| [src/path_tracking/pure_pursuit/pure_pursuit.cpp](../src/path_tracking/pure_pursuit/pure_pursuit.cpp) | Algorithm implementation |
| [include/path_tracking/pure_pursuit/pure_pursuit_loader.hpp](../include/path_tracking/pure_pursuit/pure_pursuit_loader.hpp) | File config struct |
| [src/path_tracking/pure_pursuit/pure_pursuit_loader.cpp](../src/path_tracking/pure_pursuit/pure_pursuit_loader.cpp) | JSON config loader |
| [config/path_tracking/pure_pursuit.json](../config/path_tracking/pure_pursuit.json) | Runtime parameters |

### Configuration struct

```cpp
struct PurePursuitConfig {
    float lookahead_distance;  // base look-ahead [m]
    float lookahead_gain;      // speed multiplier [dimensionless]
    float wheelbase;           // front-to-rear axle distance [m]
};
```

`wheelbase` is populated from `VehicleData` at initialisation — it is not in the JSON config.

### `SetPath`

```cpp
void PurePursuit::SetPath(const std::vector<map::Point2D>& path)
```

Stores the reference waypoint vector and resets the internal path cursor (`current_path_idx_ = 0`).

### `ComputeSteering`

```cpp
float PurePursuit::ComputeSteering(const vehicle::VehicleState& state) const
```

Full sequence:

```
1.  ld  = lookahead_distance + lookahead_gain × |vx|

2.  closest = FindClosestWaypoint(state)
              — forward window search, updates current_path_idx_

3.  target  = FindLookaheadPoint(state, closest, ld)
              — first waypoint at distance ≥ ld from vehicle

4.  dx = target.x − state.x
    dy = target.y − state.y

5.  local_y = −dx·sin(ψ) + dy·cos(ψ)
              — lateral offset of target in vehicle body frame

6.  δ = atan2(2·L·local_y, ld²)

7.  return δ  [rad]
```

### `FindClosestWaypoint`

Searches a forward window of 40 waypoints (`kSearchWindow`) starting from the last known closest index. Returns the index of the waypoint with minimum Euclidean distance to the vehicle. The cursor only moves forward — it never backtracks.

```
for i in [current_path_idx_, current_path_idx_ + 40):
    d² = (path[i].x − x)² + (path[i].y − y)²
    track minimum

current_path_idx_ = index of minimum
```

### `FindLookaheadPoint`

Scans forward from `closest` and returns the first waypoint whose distance from the vehicle is `≥ ld`. If the end of the path is reached, returns the last waypoint.

```
for i in [closest, path.size()):
    if distance(path[i], vehicle) ≥ ld:
        return path[i]
return path.back()
```

---

## Parameter Effects

### `lookahead_distance` (base look-ahead)

| Value | Effect |
|-------|--------|
| Larger | Smoother path, larger steady-state cross-track error on tight curves |
| Smaller | Tighter tracking, more steering oscillation, risk of instability |

### `lookahead_gain` (speed scaling)

| Value | Effect |
|-------|--------|
| Larger | Look-ahead grows faster with speed → smoother at high speed |
| Smaller | Look-ahead stays short even at high speed → more responsive but noisier |
| 0 | Fixed look-ahead regardless of speed |

### Combined effect at current defaults

```
Speed 0 m/s  →  Ld = 5.0 m   (tight tracking at low speed)
Speed 5 m/s  →  Ld = 7.5 m   (smoother tracking at cruise speed)
Speed 10 m/s →  Ld = 10.0 m  (conservative at high speed)
```

---

## Stability and Limitations

**Stability** — The algorithm is provably stable for a vehicle following a straight line: the cross-track error decays exponentially with a time constant proportional to `Ld / vx`. On curved paths, a residual steady-state error exists that decreases as `Ld` decreases.

**Limitations of the current implementation:**

1. **No heading look-ahead** — Only lateral error is corrected; the approach angle to the path is not directly controlled.
2. **Discrete waypoint search** — `FindLookaheadPoint` picks an exact waypoint rather than interpolating on the segment, which can cause small discretisation errors.
3. **No backward driving** — `FindClosestWaypoint` is monotonically forward; reverse path tracking is not supported.
4. **No obstacle awareness** — The controller tracks the pre-computed path blindly. Obstacle avoidance must be handled by the path planner upstream.

---

## Integration with the Simulation

The Pure Pursuit controller is called once per simulation frame inside `TickSimulation`:

```cpp
// vehicle_control_ui.cpp — TickSimulation
float delta   = pure_pursuit_.ComputeSteering(vehicle_state_);
float accel   = 1.0f * (max_speed_ - vehicle_state_.speed);

VehicleControls controls{ delta, accel };
vehicle_state_ = bicycle_model_.Step(vehicle_state_, controls, dt);
```

The returned steering angle `delta` is passed directly to the bicycle model as the desired front-wheel angle. The model internally clamps it to the vehicle's physical steering limit before integration.

The look-ahead point retrieved via `GetLookaheadPoint()` is also rendered as a yellow dot connected by a light-blue line from the rear axle, giving visual feedback of what the controller is currently targeting.

---

## Configuration File

`config/path_tracking/pure_pursuit.json`

```json
{
  "lookahead_distance": 5.0,
  "lookahead_gain": 0.5,
  "max_speed": 5.0,
  "search_window": 40
}
```

`max_speed` is not a controller parameter — it is used by the proportional speed controller in the UI loop to set the target cruise speed.

`search_window` defines how many waypoints ahead are scanned when searching for the closest point and the lookahead target. It is injected into `PurePursuitConfig` from the JSON (and the live tuning UI); it is not a field of the base algorithm class.

---

## Live Parameter Tuning

All parameters in this file can be adjusted at runtime without recompiling or restarting the simulation:

1. Enter the simulation screen and click **Start Simulation**.
2. In the right panel, check **Override Defaults** under **Parameter Tuning**.
3. Drag the desired slider; the new value takes effect the moment you release the mouse button.

| Slider | Range | Step |
|--------|-------|------|
| Lookahead Dist | 0.1 – 10.0 m | 0.1 m |
| Lookahead Gain | 0.1 – 1.0 | 0.1 |
| Max Speed | 0.1 – 20.0 m/s | 0.1 m/s |
| Search Window | 20 – 80 | 1 |

---

## Quick Reference

```
Steering formula:
  δ = atan2(2·L·e, Ld²)

Look-ahead distance:
  Ld = lookahead_distance + lookahead_gain × |vx|

Lateral error (body frame):
  e = −Δx·sin(ψ) + Δy·cos(ψ)
  where (Δx, Δy) = look-ahead point − vehicle rear axle

Waypoint search window:  40 waypoints forward from last closest index
```
