# Vehicle Control Algorithms

A desktop simulation environment for visualizing and testing vehicle path-tracking algorithms.
Built with C++17, Dear ImGui, ImPlot, GLFW, and OpenGL 3.3.

---

## Features

- **Main screen** — select a vehicle model, a path-tracking algorithm, and a map before launching the simulation.
- **Simulation screen** — displays the loaded map (obstacles, start, goal) and the pre-computed reference path on an interactive Cartesian plot.
- **JSON-driven configuration** — vehicles, maps, and paths are defined in plain JSON files; no recompilation needed to add new scenarios.
- **Auto-discovery** — the UI automatically lists all vehicle files and map directories found under `data/`.

---

## Project Structure

```
VehicleControlAlgorithms/
├── CMakeLists.txt
├── data/
│   ├── maps/
│   │   ├── map_01/          # Corridor Map  (straight corridor with gaps)
│   │   │   ├── map.json
│   │   │   └── path.json
│   │   └── map_02/          # Sine Wave Course  (two-period sinusoidal weave)
│   │       ├── map.json
│   │       └── path.json
│   └── vehicle/
│       └── vehicle01.json   # Bus 01 dynamics & geometry
├── include/
│   ├── assets/
│   │   ├── fonts/           # Nasalization TrueType font
│   │   └── icons/           # Dripicons v2 font + dripicon_v2_icons.hpp
│   ├── map/
│   │   ├── map_data.hpp     # MapData, PathData, Point2D, Obstacle structs
│   │   └── map_loader.hpp   # MapLoader — loads map.json + path.json, discovers maps
│   ├── path_tracking/
│   │   └── pure_pursuit/
│   │       └── pure_pursuit.hpp   # PurePursuit class
│   ├── vehicle/
│   │   ├── vehicle_data.hpp       # VehicleData struct (dynamics + geometry)
│   │   └── vehicle_loader.hpp     # VehicleLoader — loads vehicle JSON, discovers vehicles
│   ├── third_parties/       # ImGui, ImPlot, stb_image (vendored)
│   └── vehicle_control_ui.hpp
└── src/
    ├── main.cpp
    ├── vehicle_control_ui.cpp
    ├── map/
    │   └── map_loader.cpp
    ├── path_tracking/
    │   └── pure_pursuit/
    │       └── pure_pursuit.cpp
    └── vehicle/
        └── vehicle_loader.cpp
```

---

## Dependencies

| Library | Version | Notes |
|---------|---------|-------|
| GLFW    | 3.x     | `sudo apt install libglfw3-dev` |
| OpenGL  | 3.3+    | Usually provided by GPU driver |
| Dear ImGui | 1.9x | Vendored under `include/third_parties/imgui/` |
| ImPlot  | 0.17+   | Vendored under `include/third_parties/implot/` |
| stb_image | —    | Vendored under `include/third_parties/stb/` |

---

## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
./vehicle_control_algorithms
```

---

## Adding a New Map

1. Create a directory under `data/maps/`, e.g. `data/maps/map_03/`.
2. Add `map.json` following the schema:

```json
{
  "name": "My Map",
  "world": { "width": 12.0, "height": 6.0 },
  "start": { "x": 0.5, "y": 3.0 },
  "goal":  { "x": 11.5, "y": 3.0 },
  "vehicle_radius": 0.2,
  "obstacles": [
    { "id": "wall_a", "shape": "rectangle", "x": 2.0, "y": 0.0, "width": 0.5, "height": 3.0 }
  ]
}
```

3. Add `path.json` in the same directory:

```json
{
  "map_name": "My Map",
  "algorithm": "A*",
  "waypoints": [
    { "x": 0.5, "y": 3.0 },
    { "x": 11.5, "y": 3.0 }
  ]
}
```

The map will appear automatically in the **Select Map** popup at next launch.

---

## Adding a New Vehicle

Create a JSON file under `data/vehicle/`, e.g. `data/vehicle/vehicle02.json`:

```json
{
  "name": "My Vehicle",
  "mass": 1500.0,
  "a": 1.2,
  "b": 1.3,
  "CA": 0.00045,
  "minimum_turning_radius": 5.5,
  "max_steering_wheel_angle": 720.0,
  "min_steering_wheel_angle": -720.0,
  "max_steering_wheel_rate": 400.0,
  "wheelbase": 2.5,
  "wheel_radius": 0.32,
  "wheel_width": 0.20,
  "wheel_tread": 1.55,
  "front_overhang": 0.85,
  "rear_overhang": 0.75,
  "left_overhang":  0.10,
  "right_overhang": 0.10,
  "vehicle_height": 1.45
}
```

| Field | Description | Unit |
|-------|-------------|------|
| `mass` | Vehicle mass | kg |
| `a` | Front axle to centre of gravity | m |
| `b` | Rear axle to centre of gravity (`a + b = wheelbase`) | m |
| `CA` | Mass-normalised aerodynamic drag coefficient | 1/m |
| `minimum_turning_radius` | Minimum kinematic turning radius | m |
| `max/min_steering_wheel_angle` | Steering wheel travel limits | deg |
| `max_steering_wheel_rate` | Maximum steering rate | deg/s |
| `wheelbase` | Front-to-rear axle distance | m |
| `wheel_radius` | Loaded tyre radius | m |
| `wheel_width` | Tyre section width | m |
| `wheel_tread` | Left-to-right wheel-centre distance | m |
| `front/rear_overhang` | Axle centre to vehicle body end | m |
| `left/right_overhang` | Wheel centre to vehicle body side | m |
| `vehicle_height` | Overall height | m |

---

## Adding a New Path-Tracking Algorithm

1. Create `include/path_tracking/<algo>/<algo>.hpp` and `src/path_tracking/<algo>/<algo>.cpp`.
2. Add the source file to `CMakeLists.txt`.
3. Add the display name to `kAvailableAlgorithms` in `src/vehicle_control_ui.cpp`.

---

## Included Maps

| Map | Description |
|-----|-------------|
| **map_01** — Corridor Map | Straight corridor with three pairs of offset wall segments. Tests basic gap navigation. |
| **map_02** — Sine Wave Course | Two-period sinusoidal weave (amplitude ±1.5 m, period 5.5 m). Four alternating baffles force the path high and low. |

---

## Included Algorithms

| Algorithm | Status |
|-----------|--------|
| Pure Pursuit | Implemented — `PurePursuit::ComputeSteering()` returns wheel angle from look-ahead point |

---

## Will Be Added in Future

- [ ] **Live simulation** — tick-by-tick vehicle dynamics integration, vehicle moves along path in real time
- [ ] **Vehicle body rendering** — draw the vehicle footprint (wheelbase + overhangs) on the map as it moves
- [ ] **Telemetry panel** — real-time display of speed, steering angle, lateral error, and look-ahead point in the right panel
- [ ] **More path-tracking algorithms** — Stanley controller, MPC (Model Predictive Control), LQR
- [ ] **Path planning** — generate paths automatically (A*, RRT, Hybrid A*) instead of loading pre-computed JSON waypoints
- [ ] **Algorithm parameter tuning** — sliders/inputs for look-ahead distance, wheelbase override, gain values
- [ ] **Obstacle shapes** — support circle and polygon shapes in addition to axis-aligned rectangles
- [ ] **Multiple vehicle comparison** — run two algorithms side-by-side on the same map
- [ ] **Export / Save results** — write simulation trajectory and telemetry to CSV or JSON
- [ ] **Map editor** — draw obstacles and waypoints interactively in the simulation screen
