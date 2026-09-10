# Phase 3I: Vehicle-Level Performance & Handling Analysis

## 1. Purpose
The purpose of Phase 3I is to establish a vehicle-level performance and handling analysis layer on top of the validated Phase 3F–3H physics and simulation stack. This layer extracts, calculates, characterizes, and verifies physically meaningful vehicle-level quantities from time-domain simulation outputs.

> [!IMPORTANT]
> **Research Integrity Statement:** This phase does not introduce any new tire or vehicle physics. All results are explicitly characterized as mathematical metrics of the implemented multi-body/tire models. The representative parameters used in maneuvers are for analysis purposes and are **not** experimental measurements. The handling metrics and tire utilization metrics do not constitute a claim of commercial rover validation, nor do they represent a claimed physical "friction-circle law".

## 2. Architecture and Relationship to Phases 3F–3H
Phase 3I serves as the top-level analytical layer in the modeling stack:
1. **Tire Physics (1A-3C)**
2. **Dynamic Load Transfer (3D)**
3. **Closed-Loop Force Balance (3E-3F)**
4. **Time-Domain Trajectory Integration (3G-3H)**
5. **Phase 3I Analysis Layer**

This architecture strictly consumes the existing validated outputs (`time`, `stateHistory`, `diagnostics`) from the `simulateVehicleTrajectory` module.

## 3. State Diagnostic Equations
The state metrics module (`vehicleStateMetrics.m`) extracts the base 6-state variables $[X, Y, \psi, v_x, v_y, r]^T$.

### 3.1 Speed and Sideslip
- **Ground Speed:** $V = \sqrt{v_x^2 + v_y^2}$
- **Sideslip Angle:** $\beta = \text{atan2}(v_y, v_x)$

### 3.2 Body-Frame Accelerations
Physical body-frame accelerations (including centripetal/Coriolis effects) are computed using numerical differentiation of the state history to match the definitions established in Phase 3F:
- **Longitudinal Acceleration:** $a_x = \dot{v}_x - v_y \cdot r$
- **Lateral Acceleration:** $a_y = \dot{v}_y + v_x \cdot r$
- **Yaw Acceleration:** $\ddot{\psi} = \dot{r}$

### 3.3 Curvature
Path curvature is defined for sufficiently non-zero speeds to prevent singularity:
- **Curvature:** $\kappa_{path} = \frac{r}{V}$ (for $V > \epsilon$)

## 4. Tire-Force Diagnostics
The tire force metrics module (`tireForceMetrics.m`) processes the four-wheel force outputs provided by the trajectory diagnostics.

### 4.1 Tire Utilization Definition
Normalized utilization metrics (friction-circle proxies) are calculated as dimensionless diagnostic tools characterizing how much of the available vertical load is being converted into shear force:
- **Longitudinal Utilization:** $\mu_x = \left| \frac{F_x}{F_z} \right|$
- **Lateral Utilization:** $\mu_y = \left| \frac{F_y}{F_z} \right|$
- **Combined Utilization:** $\mu_{combined} = \sqrt{\mu_x^2 + \mu_y^2}$

## 5. Handling Metrics
The handling metrics module (`vehicleHandlingMetrics.m`) calculates specific gains and dynamic characteristics derived from the state and force metrics.
- **Yaw-Rate Gain:** $G_r = \frac{r}{\delta}$
- **Lateral-Acceleration Gain:** $G_{ay} = \frac{a_y}{\delta}$

> [!NOTE]
> Handling gains are only evaluated when the steering input $\delta$ is sufficiently non-zero to avoid numerical instability.

Additionally, this module extracts scalar summary peak metrics such as peak speed, peak accelerations, peak yaw rate, and peak tire utilization over a given maneuver.

## 6. Maneuver Definitions
The analysis driver (`runVehicleManeuverAnalysis.m`) executes standardized, deterministic maneuvers using representative parameters:
1. **Straight Acceleration:** Zero steering, positive slip (wheel speed increasing).
2. **Straight Braking:** Zero steering, negative slip (wheel speed decreasing).
3. **Constant Cornering:** Constant step steering, constant wheel speed.
4. **Steering Sweep:** Sinusoidal steering excitation.
5. **Combined Braking + Cornering:** Step steering and negative slip simultaneously.
6. **Combined Acceleration + Cornering:** Step steering and positive slip simultaneously.

## 7. Parameter Sensitivity Methodology
Sensitivity analysis is integrated into the maneuver driver by allowing perturbation of baseline parameter values (e.g., $m$, $I_z$, $h_{CG}$). This methodology assesses how variations in physical parameters alter the mathematically derived handling outputs (e.g., peak lateral acceleration or final speed), demonstrating model responsiveness without claiming experimental equivalence.

## 8. Numerical Considerations
- Numerical differentiation (`gradient`) is used for accelerations to accommodate potentially non-uniform time grids from adaptive solvers like `ode45`.
- Thresholding guards are employed to handle singularities near zero speed (for curvature) and zero steering (for handling gains).

## 9. Verification Methodology
Phase 3I includes an independent 35-test verification suite (`verifyVehiclePerformanceAnalysis.m`). This suite explicitly verifies the deterministic bounds, physical definitions, singularity handling, force summation transformations, symmetry, qualitative maneuver behaviors, and parameter sensitivity mechanics without relying on tautological assertions.

## 10. Assumptions and Limitations
- The analysis relies on the planar 3-DOF chassis limitations of previous phases (omitting pitch, roll, and heave dynamics).
- Drivetrain dynamics (e.g., motor torque curves, wheel inertia) are excluded; inputs remain prescribed kinematics (wheel speeds and steering angles).
- The utilization metrics assume a homogeneous road surface and are not true "friction measurements."
