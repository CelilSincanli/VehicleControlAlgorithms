# LQR Path Tracking

## Concept

LQR (Linear Quadratic Regulator) is an optimal control algorithm that minimises a quadratic cost function over the state and control input. For path tracking, the controller continuously computes the optimal steering angle that drives a 5-dimensional error state to zero while keeping control effort bounded.

Unlike Pure Pursuit (which is purely geometric), LQR incorporates vehicle dynamics through a linearised state-space model, considers the rate of change of errors, and produces control actions that are provably optimal for the linearised system.

---

## State-Space Formulation

### State Vector

The 5-dimensional state vector captures both the tracking errors and their rates of change:

```
x = [ e_lat,       lateral error against the reference path [m]            ]
    [ ė_lat,       lateral error rate [m/s]                                ]
    [ e_yaw,       heading error against the path tangent [rad]            ]
    [ ė_yaw,       heading error rate [rad/s]                              ]
    [ e_speed      speed error (current speed − target speed) [m/s]        ]
```

### System Matrices

The discrete-time linearised model is built at each control step from the current speed `v` and a fixed time step `dt`:

```
A (5×5):                        B (5×2):
┌ 1  dt  0   0  0 ┐             ┌ 0      0  ┐
│ 0   0  v   0  0 │             │ 0      0  │
│ 0   0  1  dt  0 │             │ 0      0  │
│ 0   0  0   0  0 │             │ v/L    0  │
└ 0   0  0   0  1 ┘             └ 0     dt  ┘
```

`A[1][2] = v`: lateral error rate grows with yaw error at speed `v` (small-angle approximation).  
`B[3][0] = v/L`: steering input changes yaw error rate (bicycle model).  
`B[4][1] = dt`: acceleration input changes speed error.

### Lateral Error Sign Convention

Lateral error is the signed perpendicular distance from the vehicle to the reference path:
- **Positive** → vehicle is to the **left** of the path direction
- **Negative** → vehicle is to the **right** of the path direction

Given the path tangent angle `θ` at the closest waypoint and vehicle offset `(dx, dy)`:
```
e_lat = cos(θ) · dy − sin(θ) · dx
e_yaw = normalize(heading − θ)
```

---

## Cost Matrices

The LQR minimises the infinite-horizon discrete quadratic cost:

```
J = Σ [ xᵀ Q x + uᵀ R u ]
```

**Q (5×5 diagonal)** — error state penalties:

| Index | State     | Parameter | Default |
|-------|-----------|-----------|---------|
| 0     | e_lat     | `q0`      | 1.0     |
| 1     | ė_lat     | `q1`      | 1.0     |
| 2     | e_yaw     | `q2`      | 1.0     |
| 3     | ė_yaw     | `q3`      | 1.0     |
| 4     | e_speed   | `q4`      | 1.0     |

**R (2×2 diagonal)** — control effort penalties:

| Index | Input        | Parameter       | Effective value        |
|-------|--------------|-----------------|------------------------|
| 0     | Steering δ   | `r_steering`    | `r_scale × r_steering` |
| 1     | Acceleration | `r_acceleration`| `r_scale × r_acceleration` |

---

## DARE Solver

The optimal cost-to-go matrix `P` is found by solving the Discrete Algebraic Riccati Equation (DARE) iteratively:

```
P₀ = Q
P_{k+1} = AᵀP_kA − AᵀP_kB(R + BᵀP_kB)⁻¹BᵀP_kA + Q
```

### Key Insight: Only a 2×2 Inverse is Needed

The matrix to invert at each iteration is `(R + BᵀP_kB)`, which is **always 2×2** because R is 2×2 and B is 5×2. A 2×2 inverse has a closed-form solution via the determinant — no LU decomposition or external matrix library is required.

```
S = R + BᵀPB          (2×2)
S⁻¹ = (1/det(S)) · [S₁₁  −S₀₁]
                    [−S₁₀   S₀₀]
```

Convergence is checked as `max|P_{k+1} − P_k| < threshold` (configurable).

---

## LQR Gain and Control Law

After DARE converges, the optimal gain matrix K (2×5) is:

```
K = (BᵀPB + R)⁻¹ · BᵀPA
```

K[0] (first row) maps the state vector to a steering correction.  
K[1] (second row) maps the state vector to an acceleration correction — this row is not used because speed is controlled separately by the simulation's proportional speed controller.

### Steering Output

The total steering angle combines feedforward and feedback terms:

```
δ_ff = atan2(L · κ, 1)        // feedforward: path curvature κ (Menger formula)
δ_fb = −K[0] · x              // feedback: optimal state correction
δ    = normalize(δ_ff + δ_fb)
```

---

## Implementation

### Files

| File | Role |
|------|------|
| [include/path_tracking/lqr/lqr.hpp](../include/path_tracking/lqr/lqr.hpp) | `LqrConfig` struct, `Lqr` class declaration |
| [src/path_tracking/lqr/lqr.cpp](../src/path_tracking/lqr/lqr.cpp) | Algorithm implementation, matrix helpers, DARE solver |
| [include/path_tracking/lqr/lqr_loader.hpp](../include/path_tracking/lqr/lqr_loader.hpp) | `LqrFileConfig`, `LoadLqrConfig()` declaration |
| [src/path_tracking/lqr/lqr_loader.cpp](../src/path_tracking/lqr/lqr_loader.cpp) | JSON config parser |
| [config/path_tracking/lqr.json](../config/path_tracking/lqr.json) | Runtime parameters |

### Class Interface

```cpp
class Lqr : public IPathTrackingAlgorithm {
    void    SetPath(const std::vector<Point2D>& waypoints) override;
    float   ComputeSteering(const vehicle::VehicleState& state) const override;
    Point2D GetLookaheadPoint() const override;  // returns reference point on path
};
```

`ComputeSteering` calls `FindClosestWaypoint` (same forward-window search as Pure Pursuit), then builds A and B from the current speed, runs the DARE solver, computes K, and returns the combined feedforward + feedback steering angle.

`GetLookaheadPoint` returns the waypoint 5 steps ahead of the closest point, used for visualisation.

### Matrix Operations

All matrix operations use fixed-size `std::array<std::array<float,N>,M>` types with `[row][col]` indexing. No heap allocation occurs in the control loop. The full set of multiply operations needed by DARE and gain computation are implemented as static helpers in the anonymous namespace of `lqr.cpp`.

---

## Configuration

Edit `config/path_tracking/lqr.json` to tune the controller without recompiling:

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `q0` | float | 1.0 | Lateral error penalty |
| `q1` | float | 1.0 | Lateral error rate penalty |
| `q2` | float | 1.0 | Yaw error penalty |
| `q3` | float | 1.0 | Yaw error rate penalty |
| `q4` | float | 1.0 | Speed error penalty |
| `r_steering` | float | 1.0 | Steering actuation cost (relative) |
| `r_acceleration` | float | 1.0 | Acceleration actuation cost (relative) |
| `r_scale` | float | 4.0 | Global R scaling factor |
| `time_step` | float | 0.016 | Fixed dt for A, B construction [s] |
| `max_speed` | float | 3.0 | Target cruise speed [m/s] |
| `dare_iterations` | int | 150 | Maximum DARE solver iterations |
| `dare_threshold` | float | 0.01 | DARE convergence threshold |

### Tuning Guidelines

- **Increase `q0`/`q2`** → tighter lateral and heading tracking, more aggressive steering
- **Increase `r_scale`** → smoother steering, less sensitive to errors
- **Decrease `r_scale`** → more responsive, may cause oscillation
- **Decrease `q1`/`q3`** → less penalty on error rates, faster initial response
- **`q4`/`r_acceleration`** affect coupling in the Riccati solution but have minimal practical impact on steering since the acceleration row of K is unused

`wheelbase` is injected from `VehicleData` at simulation start, not read from this file.

---

## Limitations

- **Fixed time step**: The A and B matrices are built with a fixed `time_step` from config rather than the actual frame time. Significant frame-rate variation can reduce optimality but not stability.
- **Speed control**: Only the first row of K (steering) is used. Acceleration is handled by the simulation's proportional speed controller.
- **Re-linearisation**: DARE is solved every tick at the current speed. This makes the controller adaptive to speed changes but adds ~0.1 ms of computation per frame (150 iterations of 5×5 operations).
- **No reverse driving**: Lateral error rate coupling `A[1][2] = v` assumes positive speed.
