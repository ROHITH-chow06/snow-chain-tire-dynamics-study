# Phase 3H: Numerical Vehicle Trajectory Integration and Analysis

## 1. Motivation & Purpose
Phase 3F established the closed-loop 3-DOF planar vehicle state derivative $[\dot{v}_x, \dot{v}_y, \dot{r}]^T$ by resolving the algebraic loop between physical accelerations $(a_x, a_y)$, dynamic vertical loads $\mathbf{F}_z$, and combined-slip tire contact forces $(\mathbf{F}_x, \mathbf{F}_y)$.

Phase 3H introduces numerical time-domain integration of these validated planar vehicle equations of motion using MATLAB's adaptive Runge-Kutta (4,5) solver (`ode45`). It couples the body-fixed dynamic derivatives to global inertial pose kinematics to produce continuous, time-resolved vehicle trajectories:
$$\mathbf{x}(t) = \begin{bmatrix} X(t) \\ Y(t) \\ \psi(t) \\ v_x(t) \\ v_y(t) \\ r(t) \end{bmatrix}$$

> [!IMPORTANT]
> **Separation of Concerns:** Phase 3H is strictly a numerical simulation and trajectory analysis layer. It performs time integration and coordinate transformation only. It does **not** introduce new tire models, new load-transfer physics, control laws, path-tracking algorithms, suspension dynamics, or wheel rotational inertia dynamics.

---

## 2. Six-State Vehicle State Vector

The planar vehicle state vector is defined as:
$$\mathbf{x} = \begin{bmatrix} X \\ Y \\ \psi \\ v_x \\ v_y \\ r \end{bmatrix} \in \mathbb{R}^6$$

| State Index | Symbol | Description | Units | Coordinate Frame |
|:---:|:---:|:---|:---:|:---|
| 1 | $X$ | Inertial longitudinal position | $\text{m}$ | Global / Inertial Frame $\mathcal{I}$ |
| 2 | $Y$ | Inertial lateral position | $\text{m}$ | Global / Inertial Frame $\mathcal{I}$ |
| 3 | $\psi$ | Vehicle yaw heading angle (unwrapped) | $\text{rad}$ | Relative to Global $+X$ axis |
| 4 | $v_x$ | Body longitudinal velocity | $\text{m/s}$ | Vehicle Body Frame $\mathcal{B}$ |
| 5 | $v_y$ | Body lateral velocity | $\text{m/s}$ | Vehicle Body Frame $\mathcal{B}$ |
| 6 | $r$ | Body yaw rate ($\dot{\psi}$) | $\text{rad/s}$ | Vehicle Body Frame $\mathcal{B}$ |

---

## 3. Coordinate Systems & Transformations

### Reference Frames
- **Global Inertial Frame ($\mathcal{I}$):** $+X$ pointing East/Forward initial datum, $+Y$ pointing North/Left, $+Z$ pointing upward.
- **Vehicle Body Frame ($\mathcal{B}$):** $+x$ pointing forward along vehicle longitudinal centerline, $+y$ pointing lateral left, $+z$ pointing upward.
- **Heading Angle ($\psi$):** Angle between global $+X$ and body $+x$, measured counter-clockwise (CCW) positive when viewed from above ($+Z$).

### Body-to-Global Velocity Transformation
The absolute velocity vector of the vehicle center of gravity (CG) expressed in the body frame is:
$$\vec{\mathbf{v}}_{\mathcal{B}} = v_x \hat{\mathbf{i}}_{\mathcal{B}} + v_y \hat{\mathbf{j}}_{\mathcal{B}}$$

Transforming into inertial coordinates via planar rotation $R_z(\psi)$:
$$\begin{bmatrix} \dot{X} \\ \dot{Y} \end{bmatrix} = \begin{bmatrix} \cos\psi & -\sin\psi \\ \sin\psi & \cos\psi \end{bmatrix} \begin{bmatrix} v_x \\ v_y \end{bmatrix}$$

Expanding explicitly:
$$\dot{X} = v_x \cos\psi - v_y \sin\psi$$
$$\dot{Y} = v_x \sin\psi + v_y \cos\psi$$

---

## 4. Yaw Kinematics
By standard rigid-body kinematics in a planar setting, the time derivative of the heading angle $\psi$ is identical to the body yaw rate $r$:
$$\dot{\psi} = r$$

> [!NOTE]
> **Heading Continuity:** The yaw angle $\psi(t)$ is integrated as a continuous real-valued state without arbitrary modulo-$2\pi$ wrapping during simulation. This prevents discontinuous step artifacts in numerical ODE integration.

---

## 5. Complete 6-State ODE System

Combining global pose kinematics with the validated Phase 3F closed-loop derivative yields the complete autonomous ODE:

$$\dot{\mathbf{x}}(t) = \frac{d}{dt}\begin{bmatrix} X \\ Y \\ \psi \\ v_x \\ v_y \\ r \end{bmatrix} = \begin{bmatrix} v_x \cos\psi - v_y \sin\psi \\ v_x \sin\psi + v_y \cos\psi \\ r \\ \dot{v}_x\big(v_x, v_y, r, \boldsymbol{\delta}(t), \boldsymbol{\omega}(t)\big) \\ \dot{v}_y\big(v_x, v_y, r, \boldsymbol{\delta}(t), \boldsymbol{\omega}(t)\big) \\ \dot{r}\big(v_x, v_y, r, \boldsymbol{\delta}(t), \boldsymbol{\omega}(t)\big) \end{bmatrix}$$

where $[\dot{v}_x, \dot{v}_y, \dot{r}]^T$ is evaluated directly via:
```matlab
sdot_3dof = closedLoopVehicleDerivative([vx; vy; r], delta(t), omega(t), params, pure_params, combined_params, solver_opts);
```

---

## 6. Input / Command Handling

Phase 3G supports open-loop prescribed control inputs:
1. **Steering Angle ($\boldsymbol{\delta}$):**
   - Scalar value $\delta$ (applied symmetrically to front steerable wheels: $\delta_{\text{FL}} = \delta_{\text{FR}} = \delta$, $\delta_{\text{RL}} = \delta_{\text{RR}} = 0$)
   - $4\times 1$ per-wheel vector $[\delta_{\text{FL}}; \delta_{\text{FR}}; \delta_{\text{RL}}; \delta_{\text{RR}}]$
   - Time-dependent function handle `@(t) delta(t)`
2. **Wheel Angular Velocities ($\boldsymbol{\omega}$):**
   - $4\times 1$ per-wheel vector $[\omega_{\text{FL}}; \omega_{\text{FR}}; \omega_{\text{RL}}; \omega_{\text{RR}}]$
   - Time-dependent function handle `@(t) omega(t)`

Wheel indexing strictly follows the standard repository convention:
- Index 1: FL (Front-Left)
- Index 2: FR (Front-Right)
- Index 3: RL (Rear-Left)
- Index 4: RR (Rear-Right)

---

## 7. Numerical Integration Architecture

### Solver Choice
The numerical integrator is MATLAB's `ode45` — an explicit Runge-Kutta (4,5) formula based on the Dormand-Prince method with embedded error estimation for adaptive step-size control. This is appropriate for the smooth, continuous nature of planar vehicle maneuvers at the trajectory timescales considered here.

> [!NOTE]
> Phase 3G (`runVehicleSimulation`) used a fixed-step RK4 integrator. Phase 3H upgrades to `ode45` with adaptive step-size control, which provides automatic error control and better accuracy guarantees across variable-speed maneuvers.

### Tolerances
- **Relative Tolerance (`RelTol`):** Default $10^{-6}$
- **Absolute Tolerance (`AbsTol`):** Default $10^{-8}$
- **Phase 3F Algebraic Loop Tolerance:** Default $10^{-6}\,\text{m/s}^2$

---

## 8. Diagnostics and Derived Quantities

When requested via `[time, state, diagnostics] = simulateVehicleTrajectory(...)`, the simulation layer evaluates the complete physical state across all output time points:
- `ax, ay`: Body longitudinal and lateral physical accelerations $[\text{m/s}^2]$
- `dot_r`: Yaw acceleration $[\text{rad/s}^2]$
- `Fx_wheel, Fy_wheel`: $N\times 4$ wheel-frame tire contact forces $[\text{N}]$
- `Fz_wheel`: $N\times 4$ dynamic normal loads $[\text{N}]$ (conserving $\sum F_{z,i} = mg$)
- `kappa, alpha`: $N\times 4$ longitudinal slip ratios and slip angles $[\text{rad}]$
- `Gxa, Gyk`: $N\times 4$ combined-slip weighting factors
- `Fx_total, Fy_total, Mz_total`: $N\times 1$ total body forces and yaw moments

---

## 9. Verification & Validation Summary

The automated verification suite `matlab/simulation/verifyVehicleTrajectorySimulation.m` validates 20 test cases:
1. **Zero-Motion Equilibrium:** Static vehicle with $\omega=0, \delta=0$ remains stationary ($X=Y=\psi=v_x=v_y=r=0$).
2. **Constant Straight-Line Motion:** Pure rolling straight trajectory exhibits $Y=0, \psi=0, r=0, X=v_x t$.
3. **Global Position Integration:** Kinematic integration matches analytical coordinates $X(t) = X_0 + v_x t, Y(t) = Y_0$.
4. **Heading Integration:** Heading $\psi(t)$ matches trapezoidal integral of yaw rate $r(t)$.
5. **Positive Steering Response:** $\delta > 0 \implies r > 0, \psi > 0, Y > 0$.
6. **Negative Steering Response:** $\delta < 0 \implies r < 0, \psi < 0, Y < 0$.
7. **Steering Symmetry:** Exact reflection symmetry under $\pm \delta$.
8. **Differential Traction Yaw:** Differential wheel speeds produce expected yaw rate and turn.
9. **Straight-Line Symmetry:** Zero lateral drift under symmetric drive conditions.
10. **State Dimensions:** Validated $N\times 1$ time and $N\times 6$ state ordering $[X, Y, \psi, v_x, v_y, r]$.
11. **Numerical Regularity:** Zero $\text{NaN}/\text{Inf}$ across nonlinear steering and throttle profiles.
12. **Solver Tolerance Sensitivity:** Strict trajectory agreement between coarse and fine ODE tolerances.
13. **Time-Resolution Consistency:** Output interpolation consistency across time grids.
14. **Deterministic Repeatability:** Exact bitwise reproducibility across identical simulation runs.
15. **Phase 3F Derivative Consistency:** Direct equivalence between 3G state derivative and Phase 3F evaluations.
16. **Non-Zero Initial Heading:** Straight-line travel along $45^\circ$ heading ray.
17. **Negative Initial Heading:** Straight-line travel along negative global axis.
18. **Time-Varying Function Handles:** Step steering and ramping wheel velocity profiles.
19. **Input Validation:** Rejection of invalid state dimensions, non-monotonic time spans, and invalid wheel inputs.
20. **Diagnostics & Conservation:** Complete verification of diagnostic matrix dimensions and vertical load conservation $\sum F_z = mg$.

---

## 10. Assumptions & Limitations

- **Planar 3-DOF Chassis:** Pitch, roll, and vertical heave dynamics are omitted. Load transfer is quasi-static.
- **Prescribed Wheel Angular Velocity:** Drivetrain dynamics, motor torque limits, and wheel rotational inertia $\dot{\omega}$ are not modeled.
- **Homogeneous Road Surface:** Per-wheel varying surface friction coefficients are deferred to future terrain modeling.
- **No Controller:** Simulation accepts prescribed open-loop inputs; closed-loop path following is left for subsequent phases.
- **Smooth Inputs Assumed:** The adaptive `ode45` solver is well-suited for smooth continuous input profiles. Discontinuous step inputs may reduce local accuracy at the discontinuity point but do not destabilize integration.

---

## 11. Future Extensions
- Closed-loop path-following and trajectory tracking controllers (Pure Pursuit, Stanley, MPC).
- Drivetrain motor torque and wheel rotational dynamics ($\dot{\omega} = \frac{T - R F_x}{I_w}$).
- Asymmetric terrain friction maps and 3D elevation kinematics.
