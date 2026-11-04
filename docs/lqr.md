# LQR Path Tracking

## Concept

LQR (Linear Quadratic Regulator) is an optimal control algorithm that minimises a quadratic cost function over the state and control input. For path tracking, the controller computes, every control period, the optimal steering *rate* that drives a 6-dimensional error state to zero while keeping control effort bounded.

Unlike Pure Pursuit (which is purely geometric), LQR incorporates vehicle dynamics through a linearised state-space model. Since the 2026-11 audit the model also contains the **steering actuator**: the wheel angle the actuator has actually reached is part of the state and the input is the steering rate. The controller therefore knows that the wheels cannot jump to a new angle, which matters for the Dynamic (Linear Tire) vehicle model whose steering is rate-limited.

---

## State-Space Formulation

### State Vector

```
x = [ e_lat,              lateral error against the reference path [m]          ]
    [ ė_lat,              lateral error rate [m/s]                              ]
    [ e_yaw,              heading error against the path tangent [rad]          ]
    [ ė_yaw,              heading error rate [rad/s]                            ]
    [ e_speed,            speed error (current speed − target speed) [m/s]      ]
    [ δ − δ_ff            applied wheel angle minus the feed-forward angle [rad] ]
```

The error rates are computed from the vehicle state, not by finite differences:

```
ė_lat = v · sin(e_yaw)
ė_yaw = r − v · κ          (r = yaw rate from VehicleState, κ = signed path curvature)
δ     = VehicleState::steering_angle   (applied by the actuator)
δ_ff  = atan(L · κ)
```

so the controller has no derivative kick after a Start or a live parameter change, and does not depend on the frame time.

### Input Vector

```
u = [ δ̇,  steering rate [rad/s]       ]
    [ a,  longitudinal acceleration   ]   (not used: speed is controlled by the simulation)
```

### System Matrices

Discrete-time model built every control period from the current speed `v` and the fixed control period `dt = 1/60 s` (the simulation's step; not user-tunable since it must match the loop):

```
A (6×6):                                       B (6×2):
┌ 1  dt  0   0    0   0   ┐                    ┌ 0         0  ┐
│ 0  0   v   v·dt 0   0   │                    │ 0         0  │
│ 0  0   1   dt   0   0   │                    │ 0         0  │
│ 0  0   0   0    0   v/L │                    │ v/L·dt    0  │
│ 0  0   0   0    1   0   │                    │ 0         dt │
└ 0  0   0   0    0   1   ┘                    └ dt        0  ┘
```

`A[1][2..3]`: the lateral error rate follows the next heading error. `A[3][5]`, `B[3][0]`: the heading error rate follows the wheel angle (bicycle model). `A[5][5]`, `B[5][0]`: the wheel angle integrates the steering rate.

### Lateral Error Sign Convention

Lateral error is the signed perpendicular distance from the rear axle to the closest path segment:
- **Positive** → vehicle is to the **left** of the path direction
- **Negative** → vehicle is to the **right** of the path direction

```
e_lat = cos(θ) · dy − sin(θ) · dx
e_yaw = normalize(heading − θ)
```

### Curvature

`κ` is the **signed** Menger curvature through the closest waypoint and its neighbours (positive = left turn, negative = right turn), so the feed-forward angle `δ_ff = atan(L·κ)` points into the bend in both directions.

---

## Cost Matrices

```
J = Σ [ xᵀ Q x + uᵀ R u ]
```

**Q (6×6 diagonal)**

| Index | State          | Parameter | Default |
|-------|----------------|-----------|---------|
| 0     | e_lat          | `q0`      | 1.0     |
| 1     | ė_lat          | `q1`      | 1.0     |
| 2     | e_yaw          | `q2`      | 1.0     |
| 3     | ė_yaw          | `q3`      | 1.0     |
| 4     | e_speed        | `q4`      | 1.0     |
| 5     | δ − δ_ff       | `q5`      | 1.0     |

**R (2×2 diagonal)**

| Index | Input          | Parameter        | Effective value             |
|-------|----------------|------------------|-----------------------------|
| 0     | Steering rate  | `r_steering`     | `r_scale × r_steering`      |
| 1     | Acceleration   | `r_acceleration` | `r_scale × r_acceleration`  |

---

## DARE Solver

The cost-to-go matrix `P` solves the Discrete Algebraic Riccati Equation, iterated as

```
P_{k+1} = AᵀP_kA − AᵀP_kB(R + BᵀP_kB)⁻¹BᵀP_kA + Q
```

The iteration is **warm-started** from the previous tick's solution. Speed changes little between frames, so it converges in a few iterations (≈ 3–4 µs per tick instead of ≈ 70 µs from a cold start). Convergence is checked as `max|P_{k+1} − P_k| < dare_threshold`.

Only a 2×2 matrix `(R + BᵀPB)` is ever inverted, in closed form.

---

## Control Law and Steering Output

```
K    = (BᵀPB + R)⁻¹ · BᵀPA                  (2×6)
δ̇*   = clamp(−K[0] · x, ±max_delta_rate)     (actuator rate of the selected vehicle model)
δcmd = clamp(δ_applied + δ̇* · dt, ±max_delta)
```

The returned command is the applied wheel angle advanced by one control period at the optimal rate, so it is always reachable by the rate-limited actuator. Under the Kinematic model (ideal steering) the same command is applied directly.

---

## Implementation

| File | Role |
|------|------|
| [include/path_tracking/lqr/lqr.hpp](../include/path_tracking/lqr/lqr.hpp) | `LqrConfig`, `Lqr` class, fixed-size matrix types |
| [src/path_tracking/lqr/lqr.cpp](../src/path_tracking/lqr/lqr.cpp) | Algorithm, templated matrix helpers, DARE |
| [include/path_tracking/lqr/lqr_loader.hpp](../include/path_tracking/lqr/lqr_loader.hpp) | `LqrFileConfig`, `LoadLqrConfig()` |
| [src/path_tracking/lqr/lqr_loader.cpp](../src/path_tracking/lqr/lqr_loader.cpp) | JSON reader (values clamped into the slider ranges) |
| [config/path_tracking/lqr.json](../config/path_tracking/lqr.json) | Runtime parameters |

`ComputeSteering` finds the closest waypoint (whole path on the first call after `SetPath`, then a forward window of `search_window`), builds A and B at the current speed, solves the DARE, computes K and returns the command above. `GetLookaheadPoint` returns the waypoint 5 steps ahead (visualisation); `GetLastTrackingDiagnostics` reports the cross-track and heading error.

All matrices are fixed-size `std::array`; no heap allocation happens in the control loop.

---

## Configuration

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `q0` … `q5` | float | 1.0 | State weights (see table above) |
| `r_steering` | float | 1.0 | Steering-rate cost (relative) |
| `r_acceleration` | float | 1.0 | Acceleration cost (relative) |
| `r_scale` | float | 4.0 | Global R scaling factor |
| `max_speed` | float | 3.0 | Target cruise speed [m/s] |
| `dare_iterations` | int | 150 | Maximum DARE iterations |
| `dare_threshold` | float | 0.01 | DARE convergence threshold |
| `search_window` | int | 40 | Waypoints scanned ahead for the closest point |

`wheelbase` and `max_delta` (tire-angle limit) come from the vehicle data; `max_delta_rate` is the actuator rate of the selected vehicle model (the vehicle's steering rate under Dynamic (Linear Tire), unlimited under the ideal Kinematic model) and is updated without resetting the controller when the model is switched mid-run. The control period comes from the simulation loop. A `time_step` key in older config files is ignored.

### Tuning Guidelines

- **Increase `q0` / `q2`** → tighter lateral and heading tracking.
- **Increase `q5`** → wheel angle stays closer to the path's feed-forward angle (calmer steering, slower correction).
- **Increase `r_scale` or `r_steering`** → slower, smoother steering.
- **`q4` / `r_acceleration`** have no practical effect on steering (the acceleration row of K is unused).

---

## Live Parameter Tuning

1. Enter the simulation screen and click **Start Simulation**.
2. Check **Override Defaults** under **Parameter Tuning**.
3. Release a slider to rebuild the controller; it keeps the vehicle's place on the path.

| Slider | Range | Step |
|--------|-------|------|
| Q0 – Q5 | 0.1 – 4.0 | 0.1 |
| R Steering / R Acceleration / R Scale | 0.1 – 4.0 | 0.1 |
| Max Speed | 0.5 – 15.0 m/s (Dynamic model: up to the path speed cap) | 0.1 m/s |
| DARE Iterations | 100 – 250 | 10 |
| DARE Threshold | 0.01 – 0.05 | 0.01 |
| Search Window | 20 – 80 | 1 |

---

## Limitations

- **Linearisation**: the model is linearised for small heading errors at the current speed; very large initial errors are handled, but not optimally.
- **Speed control**: only the steering row of K is used. Acceleration is handled by the simulation's limited speed controller.
- **No reverse driving**: `A[1][2] = v` assumes positive speed.
