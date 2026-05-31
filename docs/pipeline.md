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

1. **Compute scale factor** — `simScale = vehicle_radius / real_half_width`. All simulation coordinates operate in real metres; this scale converts to map-unit rendering coordinates.
2. **Place the vehicle** — `VehicleState` is initialised at the map's start position with zero speed and zero steering.
3. **Convert path waypoints** — reference path is scaled from map units to real metres and stored in `simScaledPath_`.
4. **Allocate trace buffers** — empty vectors for the rear-axle trail.
5. **Instantiate the vehicle model** — `KinematicBicycleModel` is created from `VehicleData`.
6. **Load JSON defaults into tuning structs** — for the selected algorithm, if its `enabled` flag is `false`, the JSON config is parsed and its values are written into the corresponding `Tune*` struct. If the user has already enabled override, the existing struct values are kept.
7. **Call `RebuildAlgorithm`** — creates the algorithm instance from the current tuning struct values.

### `RebuildAlgorithm`

Creates a new `simPursuit_` from the current `Tune*` struct and calls `SetPath`. Can be called mid-simulation without resetting vehicle state. Used in two ways:

- By `InitSimulation` at startup/restart.
- By `TickSimulation` when `paramsDirty_` is set (live parameter change).

```
Tune* struct values
        │
        ▼
  Build *Config struct  (copy Tune* fields + set wheelbase from VehicleData)
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

Called every frame (≈ 60 Hz). The time delta `dt` is computed from `glfwGetTime()` and clamped to 0.1 s to prevent instability after a pause.

```
┌──────────────────────────────────────────────────────────────┐
│  Each call to TickSimulation(dt)                             │
│                                                              │
│  0. Live rebuild check                                       │
│     if paramsDirty_ → RebuildAlgorithm()                    │
│     (parameter slider released while simulation running)    │
│                                                              │
│  1. Goal check                                               │
│     distance(vehicle, goal) < 1 m  →  state = SIM_DONE      │
│                                                              │
│  2. Compute steering                                         │
│     δ = simPursuit_->ComputeSteering(vehicle_state)          │
│                                                              │
│  3. Compute acceleration                                     │
│     a = 1.0 × (simMaxSpeed_ − current_speed)                │
│     (proportional controller on speed error)                │
│                                                              │
│  4. Build control input                                      │
│     VehicleControls { steering_angle=δ, acceleration=a }    │
│                                                              │
│  5. Integrate one step                                       │
│     next_state = bicycle_model.Step(state, controls, dt)    │
│                                                              │
│  6. Record trace                                             │
│     append (x, y) to trace buffers every 0.5 m             │
│                                                              │
│  7. Render (described in Stage 4)                            │
└──────────────────────────────────────────────────────────────┘
```

### Kinematic Bicycle Model integration

Inside `KinematicBicycleModel::Step`, equations of motion are integrated with the Euler forward method:

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
| Steering | Current front-wheel steering angle [deg] |
| Status | Running / Done / Idle |

### Right-side panel — Parameter Tuning

Rendered below the Simulation Info section. Occupies all remaining space above the bottom of the panel.

**Behaviour:**

- **Override Defaults unchecked** (default) — sliders are visible but disabled (greyed out). Values shown are the currently loaded defaults (from JSON or the last active override session). Starting a simulation with override disabled always re-reads the JSON file.
- **Override Defaults checked** — checkbox turns green; sliders become interactive. Moving a slider updates its display value immediately. The algorithm is rebuilt from the new values the moment the mouse button is released (`IsItemDeactivatedAfterEdit`), so the vehicle responds without interrupting the simulation.
- **Hover tooltip** — hovering a slider (when override is active) shows a semi-transparent description of that parameter drawn on the foreground draw list. The tooltip fades out over 0.25 s once the cursor leaves the slider.

Slider height is distributed evenly across all parameters for the active algorithm. All sliders stretch the full panel width.

### Application icon

The window icon (`include/assets/images/vca_application_icon.png`) is loaded via `stb_image` during `Initialize()` and set with `glfwSetWindowIcon`. It appears in the taskbar, the alt-tab switcher, and the window title bar on X11/GNOME desktops.

---

## Stage 5 — Termination

The loop ends when the vehicle reaches within 1 m of the goal. The simulation status changes to `SIM_DONE`. The user can click **Restart** to re-run the simulation with the same or adjusted parameters, or use the back button to return to the main menu and choose a different configuration.

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
 KinematicBicycleModel           VehicleState (init at start pos)
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
    │  VehicleState ──► SpeedController ──────────────► a │      │
    │                                                      │      │
    │  KinematicBicycleModel::Step(state, {δ, a}, dt)             │
    │              │                                               │
    │         next VehicleState ──────────────────────────────────┤
    │              │                                               │
    │         Rendering (ImPlot + ImGui right panel)               │
    └──────────────────────────────────────────────────────────────┘
```

---

## Key Design Decisions

**Rear-axle reference frame** — All vehicle position state (`x`, `y`) refers to the rear-axle centre. This is the natural origin for a bicycle model because the rear wheel has zero slip by assumption in the kinematic case.

**Euler forward integration** — Simple and stable for small `dt`. The 0.1 s cap prevents blow-up during pauses.

**Proportional speed controller** — `a = 1.0 × (v_max − v)` converges the speed to `simMaxSpeed_` without overshoot. `simMaxSpeed_` is drawn from the active tuning struct's `max_speed` field so it responds to live slider changes.

**Split Init / Rebuild** — `InitSimulation` resets vehicle state and reloads JSON defaults; `RebuildAlgorithm` only recreates the algorithm object from the current tuning struct. This separation allows live parameter changes mid-simulation without disrupting the vehicle's position or trajectory.

**`paramsDirty_` flag** — Sliders set this flag only on `IsItemDeactivatedAfterEdit` (mouse release), not during drag. This prevents the algorithm from being rebuilt tens of times per second while the user is moving a slider.

**Foreground draw list for tooltips** — The hover tooltip is drawn with `ImGui::GetForegroundDrawList()` rather than as an ImGui window. This guarantees it is always composited above all ImGui windows and child windows regardless of window ordering or simulation restarts.

**JSON-driven configuration** — Vehicles, maps, and algorithm parameters can all be modified or added without recompiling. The loaders auto-discover files by scanning directories.

**Scale factor** — The simulation operates entirely in real metres. `simScale` is only applied at the rendering boundary to size on-screen elements correctly relative to the map coordinate system.
