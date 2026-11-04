# Simulation Pipeline

This document describes the end-to-end data flow of the VehicleControlAlgorithms simulation — from loading assets on startup to the real-time control loop that drives and renders the vehicle.

---

## Overview

The application is a desktop simulator for testing vehicle path-tracking algorithms. It loads a vehicle model, a reference path, and a map from JSON configuration files, then runs a closed-loop control simulation at approximately 60 Hz while rendering the scene with Dear ImGui + ImPlot.

```
JSON configs ──► Loaders ──► Tuning structs ──► RebuildAlgorithm ──► Control loop ──► Rendering
```

---

## Stage 1 — Asset Loading

The user selects three things on the main menu screen:

| Selection | Source | Loaded by |
|-----------|--------|-----------|
| Vehicle model | `data/vehicle/*.json` | `VehicleLoader` |
| Algorithm | Hard-coded list in `kAvailableAlgorithms` | UI |
| Map + path | `data/maps/<map_dir>/map.json` + `path.json` | `MapLoader` |

### VehicleLoader (`src/vehicle/vehicle_loader.cpp`)

Scans `data/vehicle/` and parses each JSON file into a `VehicleData` struct. Parameters loaded:

```
mass, a (COG→front axle), b (COG→rear axle), CA (aero drag),
wheelbase, wheel_radius, wheel_width, wheel_tread,
front/rear/left/right overhangs, vehicle_height,
max_steering_wheel_angle, steering_wheel_to_tire_angle_ratio
```

Plus optional fields consumed only under the "Dynamic (Linear Tire)" vehicle-model fidelity (see `README.md`'s "Adding a New Vehicle" section for the full schema): `cog_height`, `yaw_inertia`, and the nested `tire`/`suspension`/`steering` blocks.

Two vehicles are currently provided:

| File | Name | Description |
|------|------|-------------|
| `vehicle01.json` | Bus 01 | 8 300 kg city bus, 4.335 m wheelbase |
| `vehicle02.json` | Car 01 | 1 500 kg passenger car, 2.6 m wheelbase |

### MapLoader (`src/map/map_loader.cpp`)

Scans `data/maps/` for subdirectories that contain both `map.json` and `path.json`. Parses them into:

- `MapData` — world dimensions, start/goal positions, obstacle list
- `PathData` — ordered waypoint array, algorithm name, map name

### Algorithm Config Loaders (`src/path_tracking/<algo>/<algo>_loader.cpp`)

Each algorithm has a dedicated loader that parses its JSON config into a `*FileConfig` struct using the shared `config_reader.hpp` helpers (`readFloat` / `readInt`). The loaders and their config files are:

| Algorithm | Loader | Config file |
|-----------|--------|-------------|
| Pure Pursuit | `LoadPurePursuitConfig` | `config/path_tracking/pure_pursuit.json` |
| Adaptive Pure Pursuit | `LoadAdaptivePurePursuitConfig` | `config/path_tracking/adaptive_pure_pursuit.json` |
| LQR | `LoadLqrConfig` | `config/path_tracking/lqr.json` |
| Stanley | `LoadStanleyConfig` | `config/path_tracking/stanley.json` |
| MPC | `LoadMpcConfig` | `config/path_tracking/mpc.json` |
| MPPI | `LoadMppiConfig` | `config/path_tracking/mppi.json` |

---

## Stage 2 — Simulation Initialisation (`InitSimulation` → `RebuildAlgorithm`)

Called when the user presses **Start** or **Restart**. Split into two functions:

### `InitSimulation`

Performs full setup — runs only on Start/Restart:

1. **Units** — maps are authored in metres, so map, path and simulation share one unit for every vehicle (`simScale_` is 1).
2. **Place the vehicle** — `VehicleState` (rear-axle pose) is initialised at the map's start position, heading along the first path segment, with zero speed and a centred wheel.
3. **Copy the path** — reference path stored in `simScaledPath_`; its length feeds the timeout.
4. **Allocate trace buffers** — empty vectors for the rear-axle trail; debug log, accumulator, elapsed time and last speed command reset.
5. **Instantiate the vehicle model** — `RebuildVehicleModel` overlays the live vehicle tuning and calls `VehicleModelFactory::MakeVehicleModel` (Kinematic or Dynamic (Linear Tire)).
6. **Load JSON defaults into tuning structs** — for the selected algorithm, if its `enabled` flag is `false`, the JSON config is read (every value clamped into its slider range; out-of-range or non-numeric values listed in a "Config file adjusted" notification) and written into the corresponding `Tune*` struct. If the user has already enabled override, the existing struct values are kept.
7. **Measure the path against the vehicle** — `RebuildVehicleModel` runs `AnalyzePathFeasibility` (tightest bend, speed cap) on the effective vehicle data.
8. **Clamp the tuning structs** — `ApplyTuningLimits` clamps every value of the active algorithm into the ranges in `tuning_limits.hpp`, with Max Delta limited to the vehicle's tire angle and, under the Dynamic model, Max Speed limited to the path's speed cap. Clamped values are listed in a notification.
9. **Call `RebuildAlgorithm`** — creates the algorithm instance from the current tuning struct values.

### `RebuildAlgorithm`

Creates a new `simPursuit_` from the current `Tune*` struct and calls `SetPath`. Can be called mid-simulation without resetting vehicle state: on its first `ComputeSteering` call every algorithm searches the whole path for the closest waypoint, so it continues from the vehicle's real position. Used in two ways:

- By `InitSimulation` at startup/restart.
- By `TickSimulation` when `paramsDirty_` is set (live parameter change).

```
Tune* struct values
        │
        ▼
  Build *Config struct  (copy Tune* fields + wheelbase and tire-angle limit from the
                         effective vehicle data, the selected model's actuator rate,
                         and the control period)
        │
        ▼
  std::make_unique<Algorithm>(cfg)
        │
        ▼
  simPursuit_->SetPath(simScaledPath_)
        │
        ▼
  paramsDirty_ = false
```

---

## Stage 3 — Simulation Loop (`TickSimulation`)

The loop is **fixed-step**: each frame adds the elapsed wall time to an accumulator (backlog capped at 0.25 s) and runs `TickSimulation(sim::kControlDt)` once per 1/60 s of accumulated time (at most 15 steps per frame). Controller and vehicle model therefore always see the same step, whatever the display frame rate, and a run is reproducible.

```
┌──────────────────────────────────────────────────────────────┐
│  Each call to TickSimulation(dt = 1/60 s)                    │
│                                                              │
│  0. Live rebuild check                                       │
│     if modelDirty_  → RebuildVehicleModel(), re-clamp params,│
│                       simPursuit_->SetActuatorRate(...)      │
│     if paramsDirty_ → RebuildAlgorithm()                     │
│                                                              │
│  1. Compute steering                                         │
│     δ = simPursuit_->ComputeSteering(vehicle_state)          │
│                                                              │
│  2. Compute acceleration (sim::SpeedCommand)                 │
│     a = clamp(1.0 × (v_target − v), −max_deceleration,       │
│               +max_acceleration), rate-limited by max_jerk   │
│                                                              │
│  3. Integrate one step                                       │
│     next_state = simModel_->Step(state, {δ, a}, dt)          │
│     (state carries the applied wheel angle)                  │
│                                                              │
│  4. Record trace (every 0.5 m) and debug sample              │
│                                                              │
│  5. Termination (sim::CheckTermination)                      │
│     goal radius 1 m, or goal line crossed near the path      │
│        → SIM_DONE                                            │
│     body hits an obstacle / leaves the map / timeout         │
│        → SIM_FAILED (reason shown in Status)                 │
└──────────────────────────────────────────────────────────────┘
```

`sim::SpeedCommand` and `sim::CheckTermination` live in `src/sim/sim_rules.cpp` and are shared with the headless test harness (`tests/sim_harness.cpp`), so tests run exactly the GUI's rules.

### Kinematic Bicycle Model integration

Inside `KinematicBicycleModel::Step`, equations of motion are integrated with the Euler forward method:

```
δ_clamped = clamp(δ, δ_min, δ_max)       (ideal, instant steering actuator)

ẋ    = vx · cos(ψ)
ẏ    = vx · sin(ψ)
ψ̇   = (vx / L) · tan(δ_clamped)
v̇x  = u_a − CA · vx·|vx|

next.x       = x  + ẋ  · dt
next.y       = y  + ẏ  · dt
next.heading = ψ  + ψ̇  · dt
next.speed   = max(0,  vx + v̇x · dt)
```

Where:
- `L` — wheelbase [m]
- `CA` — aerodynamic drag coefficient [1/m]
- `δ_max`, `δ_min` — tire-angle limits from `vehicle::TireAngleLimits`: steering-wheel travel / ratio, further limited to `atan(L / minimum_turning_radius)`

### Dynamic (Linear Tire) model integration

`ComposedVehicleModel::Step` sub-steps the given `dt` in ≤ 2 ms explicit-Euler steps (the tire dynamics are stiff at low speed). Each sub-step first runs the steering actuator (rate-limited when the vehicle file enables it), then either the kinematic fallback (below 0.5 m/s) or the dynamic bicycle equations with linear tires, longitudinal load transfer and the full projection of the front tire force. The position integrated is the **rear axle** (lateral velocity `vy − b·r`), the same reference as the Kinematic model, the controllers and the renderer. A model rebuilt mid-run continues from the wheel angle in the incoming state.

---

## Stage 4 — Rendering

Rendering happens inside the same frame as `TickSimulation`. The screen is divided into three areas:

| Area | Size | Content |
|------|------|---------|
| Header bar | 99% × 6% | Back button (returns to main menu, resets simulation) |
| Plot area | 72% × 88% | ImPlot Cartesian map — obstacles, path, vehicle, trace |
| Right panel | 25% × 88% | Simulation Info + Parameter Tuning |

### Layers drawn in the plot (back to front)

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
world_x = rear_axle_x + fwd * cos(ψ) − lat * sin(ψ)
world_y = rear_axle_y + fwd * sin(ψ) + lat * cos(ψ)
```

where `fwd` is the longitudinal offset and `lat` is the lateral offset from the rear axle.

### Right-side panel — Simulation Info

Displays in real time:

| Row | Value |
|-----|-------|
| Algorithm | Selected algorithm name |
| Vehicle Type | Loaded vehicle name |
| Map | Loaded map name |
| Speed | Current speed [km/h] |
| Steering | Commanded tire angle and, in brackets, the wheel angle actually reached [deg] |
| Status | Idle / Running / Paused / Done: goal reached / Failed: reason |

### Right-side panel — Parameter Tuning

Rendered below the Simulation Info section. Occupies all remaining space above the bottom of the panel.

**Behaviour:**

- **Override Defaults unchecked** (default) — sliders are visible but disabled (greyed out). Values shown are the currently loaded defaults (from JSON or the last active override session). Starting a simulation with override disabled always re-reads the JSON file.
- **Override Defaults checked** — checkbox turns green; sliders become interactive. Moving a slider updates its display value immediately. The algorithm is rebuilt from the new values the moment the mouse button is released (`IsItemDeactivatedAfterEdit`), so the vehicle responds without interrupting the simulation.
- **Hover tooltip** — hovering a slider (when override is active) shows a semi-transparent description of that parameter drawn on the foreground draw list. The tooltip fades out over 0.25 s once the cursor leaves the slider.

Slider height is distributed evenly across all parameters for the active algorithm. All sliders stretch the full panel width. Slider ranges come from `include/tuning_limits.hpp`; Max Delta and (Dynamic model) Max Speed upper bounds are tightened per run.

### Render cadence and resource use

`Run()` renders at the display rate only while something on screen changes: a running simulation, user input in the last 0.5 s, a fading tooltip or a visible notification. Otherwise it blocks in `glfwWaitEventsTimeout(0.25)`, so an idle window uses about 0.1 % of one core instead of ~3 %. The debug log keeps at most 36 000 samples (10 minutes at 60 Hz, ~4 MB) and the trace at most 20 000 points; the reference-path plot arrays are built once per map load. The Simulation Info panel shows frame rate, controller + vehicle-model time per tick, and resident memory.

### Application icon

The window icon (`include/assets/images/vca_application_icon.png`) is loaded via `stb_image` during `Initialize()` and set with `glfwSetWindowIcon`. It appears in the taskbar, the alt-tab switcher, and the window title bar on X11/GNOME desktops.

---

## Stage 5 — Termination

The run ends when `sim::CheckTermination` reports an outcome:

| Outcome | Condition | Status shown |
|---------|-----------|--------------|
| Goal | rear axle within 1 m of the goal, or across the goal line (last path segment) within 3 m of the path and 10 m of the goal | Done: goal reached |
| Collision | the body rectangle overlaps an obstacle | Failed: collision |
| Off map | a body corner leaves the world | Failed: left the map |
| Timeout | elapsed time > 3 × path length / target speed + 30 s | Failed: timeout |

The user can click **Restart** to re-run with the same or adjusted parameters, or use the back button to return to the main menu.

---

## Data Flow Diagram

```
                    data/vehicle/*.json
                           │
                    VehicleLoader
                           │
                      VehicleData
                           │
          ┌────────────────┴─────────────────┐
          │                                   │
 MakeVehicleModel(fidelity)      VehicleState (init at start pos)
          │
          │       config/path_tracking/<algo>.json
          │                    │
          │           <Algo>Loader
          │                    │
          │         <Algo>FileConfig
          │                    │
          │              ┌─────▼──────────────────────┐
          │              │  Tune* struct               │
          │              │  (if !enabled: copy from    │
          │              │   FileConfig; else keep UI  │
          │              │   override values)          │
          │              └─────┬──────────────────────┘
          │                    │
          │           RebuildAlgorithm()
          │                    │
          │   data/maps/<map>/path.json
          │                    │
          │              MapLoader
          │                    │
          │   ┌───────── PathData (waypoints)
          │   │
          │   simPursuit_->SetPath()
          │
    ┌──────────────────── TickSimulation(dt) ─────────────────────┐
    │                                                              │
    │  if paramsDirty_ → RebuildAlgorithm()  (live slider update) │
    │                                                              │
    │  VehicleState ──► simPursuit_->ComputeSteering() ──► δ      │
    │                                                      │      │
    │  VehicleState ──► sim::SpeedCommand ────────────► a │      │
    │                                                      │      │
    │  simModel_->Step(state, {δ, a}, dt)   (fixed dt = 1/60 s)   │
    │              │                                               │
    │         next VehicleState ──────────────────────────────────┤
    │              │                                               │
    │         Rendering (ImPlot + ImGui right panel)               │
    └──────────────────────────────────────────────────────────────┘
```

---

## Key Design Decisions

**Rear-axle reference frame** — All vehicle position state (`x`, `y`) refers to the rear-axle centre. This is the natural origin for a bicycle model because the rear wheel has zero slip by assumption in the kinematic case.

**Fixed-step integration** — The loop advances in 1/60 s steps from an accumulator; the Dynamic model sub-steps at ≤ 2 ms, which keeps explicit Euler stable for the stiff low-speed tire dynamics.

**Limited speed controller** — `a = 1.0 × (v_max − v)`, clamped to the vehicle's `max_acceleration` / `max_deceleration` and rate-limited by `max_jerk`. `simMaxSpeed_` is drawn from the active tuning struct's `max_speed` field so it responds to live slider changes.

**Split Init / Rebuild** — `InitSimulation` resets vehicle state and reloads JSON defaults; `RebuildAlgorithm` only recreates the algorithm object from the current tuning struct. This separation allows live parameter changes mid-simulation without disrupting the vehicle's position or trajectory.

**`paramsDirty_` flag** — Sliders set this flag only on `IsItemDeactivatedAfterEdit` (mouse release), not during drag. This prevents the algorithm from being rebuilt tens of times per second while the user is moving a slider.

**Foreground draw list for tooltips** — The hover tooltip is drawn with `ImGui::GetForegroundDrawList()` rather than as an ImGui window. This guarantees it is always composited above all ImGui windows and child windows regardless of window ordering or simulation restarts.

**JSON-driven configuration** — Vehicles, maps, and algorithm parameters can all be modified or added without recompiling. The loaders auto-discover files by scanning directories.

**Metres everywhere** — Maps, paths, vehicle state and rendering all use metres, so a map is the same physical course for every vehicle.
