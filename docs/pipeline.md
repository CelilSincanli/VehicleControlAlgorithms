# Simulation Pipeline

This document describes the end-to-end data flow of the VehicleControlAlgorithms simulation — from loading assets on startup to the real-time control loop that drives and renders the vehicle.

---

## Overview

The application is a desktop simulator for testing vehicle path-tracking algorithms. It loads a vehicle model, a reference path, and a map from JSON configuration files, then runs a closed-loop control simulation at approximately 60 Hz while rendering the scene with Dear ImGui + ImPlot.

```
JSON configs ──► Loaders ──► Simulation state ──► Control loop ──► Rendering
```

---

## Stage 1 — Asset Loading

The user selects three things on the main menu screen:

| Selection | Source | Loaded by |
|-----------|--------|-----------|
| Vehicle model | `data/vehicle/*.json` | `VehicleLoader` |
| Algorithm | Hard-coded list (currently Pure Pursuit only) | UI |
| Map + path | `data/maps/<map_dir>/map.json` + `path.json` | `MapLoader` |

### VehicleLoader (`src/vehicle/vehicle_loader.cpp`)

Scans `data/vehicle/` and parses each JSON file into a `VehicleData` struct. All 11 parameters are loaded:

```
mass, a (COG→front axle), b (COG→rear axle), CA (aero drag),
wheelbase, wheel_radius, wheel_width, wheel_tread,
front/rear/left/right overhangs, vehicle_height,
max_steering_wheel_angle, steering_wheel_to_tire_angle_ratio
```

### MapLoader (`src/map/map_loader.cpp`)

Scans `data/maps/` for subdirectories that contain both `map.json` and `path.json`. Parses them into:

- `MapData` — world dimensions, start/goal positions, obstacle list
- `PathData` — ordered waypoint array, algorithm name, map name

### PurePursuitLoader (`src/path_tracking/pure_pursuit/pure_pursuit_loader.cpp`)

Parses `config/path_tracking/pure_pursuit.json` into `PurePursuitFileConfig`:

```json
{
  "lookahead_distance": 5.0,
  "lookahead_gain":     0.5,
  "max_speed":          5.0
}
```

---

## Stage 2 — Simulation Initialization (`InitSimulation`)

Called once when the user presses **Start** on the main menu. Steps in order:

1. **Instantiate the vehicle model** — `KinematicBicycleModel` is created with the loaded `VehicleData`.
2. **Instantiate the controller** — `PurePursuit` is created with a `PurePursuitConfig` that takes `lookahead_distance`, `lookahead_gain`, and `wheelbase` from the loaded data.
3. **Load the reference path** — `PurePursuit::SetPath()` receives the waypoint vector from `PathData`.
4. **Compute scale factor** — `simScale = vehicle_radius / real_half_width`. All simulation coordinates are already in real meters; this scale is only applied at the rendering boundary.
5. **Place the vehicle** — `VehicleState` is initialised at the map's start position with zero speed.
6. **Allocate trace buffers** — Empty vectors for recording the rear-axle trail.

---

## Stage 3 — Simulation Loop (`TickSimulation`)

Called every frame (≈ 60 Hz). The time delta `dt` is computed from `glfwGetTime()` and clamped to 0.1 s to prevent instability after a pause.

```
┌──────────────────────────────────────────────────────────┐
│  Each call to TickSimulation(dt)                         │
│                                                          │
│  1. Goal check                                           │
│     distance(vehicle, goal) < 1 m  →  state = SIM_DONE  │
│                                                          │
│  2. Compute steering                                     │
│     δ = PurePursuit::ComputeSteering(vehicle_state)      │
│                                                          │
│  3. Compute acceleration                                 │
│     a = 1.0 × (max_speed − current_speed)               │
│     (proportional controller on speed error)            │
│                                                          │
│  4. Build control input                                  │
│     VehicleControls { steering_angle=δ, acceleration=a } │
│                                                          │
│  5. Integrate one step                                   │
│     next_state = bicycle_model.Step(state, controls, dt) │
│                                                          │
│  6. Record trace                                         │
│     append (x, y) to trace buffers every 0.5 m          │
│                                                          │
│  7. Render (described in Stage 4)                        │
└──────────────────────────────────────────────────────────┘
```

### Kinematic Bicycle Model integration

Inside `KinematicBicycleModel::Step`, the equations of motion are integrated with Euler forward method:

```
δ_clamped = clamp(δ, −δ_max, +δ_max)

ẋ    = vx · cos(ψ)
ẏ    = vx · sin(ψ)
ψ̇   = (vx / L) · tan(δ_clamped)
v̇x  = u_a − CA · vx²

next.x       = x  + ẋ  · dt
next.y       = y  + ẏ  · dt
next.heading = ψ  + ψ̇  · dt
next.speed   = max(0,  vx + v̇x · dt)
```

Where:
- `L` — wheelbase [m]
- `CA` — aerodynamic drag coefficient [1/m]
- `δ_max` — max tire angle = `max_steering_wheel_angle / steering_ratio`

---

## Stage 4 — Rendering

Rendering happens inside the same frame as `TickSimulation`. The ImPlot window is a Cartesian plot locked to the map's world dimensions with an equal aspect ratio.

### Layers drawn (back to front)

| Layer | Description |
|-------|-------------|
| Obstacles | Filled grey rectangles with borders |
| Reference path | Green polyline through all waypoints |
| Start marker | Green circle at start position |
| Goal marker | Red square at goal position |
| Trace trail | Blue dots at every recorded rear-axle position |
| Lookahead line | Light-blue line from rear axle to lookahead target |
| Lookahead target | Yellow filled circle |
| Vehicle body | Blue filled rectangle (front/rear overhangs + width) |
| Rear tires | Two black rectangles aligned with body heading |
| Front tires | Two black rectangles rotated by steering angle δ |

### Coordinate transformation

All vehicle geometry is expressed in the rear-axle body frame and transformed to world (plot) coordinates with:

```cpp
world_x = rear_axle_x + fwd * cos(ψ) + lat * (−sin(ψ))
world_y = rear_axle_y + fwd * sin(ψ) + lat *   cos(ψ)
```

where `fwd` is longitudinal offset and `lat` is lateral offset from the rear axle.

### Right-side status panel

Displays in real time:
- Current speed [m/s]
- Active algorithm name
- Vehicle type
- Simulation status (RUNNING / DONE)

---

## Stage 5 — Termination

The loop ends when the vehicle reaches within 1 m of the goal. The simulation status changes to `SIM_DONE`. The user can return to the main menu to select a different configuration and restart.

---

## Data Flow Diagram

```
                    data/vehicle/*.json
                           │
                    VehicleLoader
                           │
                      VehicleData
                           │
                  ┌────────┴────────┐
                  │                 │
         KinematicBicycleModel   VehicleState (init at start pos)
                  │
                  │         config/path_tracking/pure_pursuit.json
                  │                       │
                  │             PurePursuitLoader
                  │                       │
                  │             PurePursuitFileConfig
                  │                       │
                  │         data/maps/<map>/path.json
                  │                       │
                  │                  MapLoader
                  │                       │
                  │   ┌───────────── PathData (waypoints)
                  │   │
                  │   PurePursuit::SetPath()
                  │
                  │
    ┌─────────────────────── TickSimulation(dt) ──────────────────┐
    │                                                              │
    │  VehicleState ──► PurePursuit::ComputeSteering() ──► δ      │
    │                                                      │      │
    │  VehicleState ──► SpeedController ──────────────► a │      │
    │                                                      │      │
    │  KinematicBicycleModel::Step(state, {δ, a}, dt)             │
    │              │                                               │
    │         next VehicleState ──────────────────────────────────┤
    │              │                                               │
    │         Rendering (ImPlot)                                   │
    └──────────────────────────────────────────────────────────────┘
```

---

## Key Design Decisions

**Rear-axle reference frame** — All vehicle position state (`x`, `y`) refers to the rear-axle center. This is the natural origin for a bicycle model because the rear wheel has zero slip by assumption in the kinematic case.

**Euler forward integration** — Simple and stable for small `dt`. The frame cap of 0.1 s prevents blow-up during pauses.

**Proportional speed controller** — `a = 1.0 × (v_max − v)` converges the speed to `max_speed` without overshoot. No dedicated PID is needed for this use case.

**JSON-driven configuration** — Vehicles, maps, and algorithm parameters can all be modified or added without recompiling. The loaders auto-discover files by scanning directories.

**Scale factor** — Simulation operates entirely in real meters. `simScale` is only applied at the rendering boundary to size on-screen elements correctly relative to the map.
