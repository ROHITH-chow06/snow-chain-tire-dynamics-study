# Phase 3G: Time-Domain Vehicle Simulation Framework

## 1. Objective & Overview
Phase 3G establishes a complete time-domain numerical simulation framework that integrates the closed-loop 3-DOF planar vehicle dynamics developed across Phases 3A–3F.

By combining the self-consistent force-balance derivative with inertial pose kinematics, the simulation evolves the vehicle state forward in time under prescribed steering angles $\delta(t)$ and wheel angular velocities $\boldsymbol{\omega}(t)$.

---

## 2. State Equations

The planar vehicle simulation state vector is defined as:
$$\mathbf{x}(t) = \begin{bmatrix} X(t) \\ Y(t) \\ \psi(t) \\ v_x(t) \\ v_y(t) \\ r(t) \end{bmatrix} \in \mathbb{R}^6$$

where:
- $X$: Global inertial longitudinal position $[\text{m}]$
- $Y$: Global inertial lateral position $[\text{m}]$
- $\psi$: Vehicle yaw heading angle $[\text{rad}]$ (continuous, unwrapped)
- $v_x$: Vehicle body-fixed longitudinal velocity $[\text{m/s}]$
- $v_y$: Vehicle body-fixed lateral velocity $[\text{m/s}]$
- $r$: Vehicle body-fixed yaw rate $[\text{rad/s}]$

### Kinematic Mapping (World-to-Body Transformation)
$$\dot{X} = v_x \cos\psi - v_y \sin\psi$$
$$\dot{Y} = v_x \sin\psi + v_y \cos\psi$$
$$\dot{\psi} = r$$

### Dynamic Mapping (Phase 3F Closed-Loop Derivative)
$$[\dot{v}_x, \dot{v}_y, \dot{r}]^T = \text{closedLoopVehicleDerivative}([v_x; v_y; r], \delta(t), \boldsymbol{\omega}(t), \text{params})$$

The complete continuous ODE system is:
$$\dot{\mathbf{x}}(t) = \mathbf{f}(t, \mathbf{x}) = \begin{bmatrix} v_x \cos\psi - v_y \sin\psi \\ v_x \sin\psi + v_y \cos\psi \\ r \\ \dot{v}_x(v_x, v_y, r, \delta(t), \boldsymbol{\omega}(t)) \\ \dot{v}_y(v_x, v_y, r, \delta(t), \boldsymbol{\omega}(t)) \\ \dot{r}(v_x, v_y, r, \delta(t), \boldsymbol{\omega}(t)) \end{bmatrix}$$

---

## 3. Coordinate Systems & Conventions
- **Global Inertial Frame ($\mathcal{I}$):** $+X$ forward/East reference, $+Y$ left/North reference, $+Z$ upward.
- **Vehicle Body Frame ($\mathcal{B}$):** $+x$ along vehicle forward longitudinal axis, $+y$ along vehicle lateral left axis, $+z$ upward.
- **Heading Angle ($\psi$):** Angle from global $+X$ to body $+x$, positive counter-clockwise (CCW) viewed from above.
- **Wheel Indexing:** $[1: \text{FL}, 2: \text{FR}, 3: \text{RL}, 4: \text{RR}]$.

---

## 4. Numerical Integration: Classic Runge-Kutta 4th-Order (RK4)

The framework employs a deterministic fixed-step RK4 integrator (`runVehicleSimulation.m`).

Given the state $\mathbf{x}_n$ at time $t_n$ and fixed time step $h = \Delta t$:
1. $\mathbf{k}_1 = \mathbf{f}(t_n, \mathbf{x}_n)$
2. $\mathbf{k}_2 = \mathbf{f}\left(t_n + \frac{h}{2}, \mathbf{x}_n + \frac{h}{2}\mathbf{k}_1\right)$
3. $\mathbf{k}_3 = \mathbf{f}\left(t_n + \frac{h}{2}, \mathbf{x}_n + \frac{h}{2}\mathbf{k}_2\right)$
4. $\mathbf{k}_4 = \mathbf{f}(t_n + h, \mathbf{x}_n + h \mathbf{k}_3)$
5. $\mathbf{x}_{n+1} = \mathbf{x}_n + \frac{h}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)$

### Integration Accuracy & Order
- Local truncation error: $\mathcal{O}(h^5)$
- Global accumulated error: $\mathcal{O}(h^4)$
- For typical rover dynamics with step size $\Delta t \in [0.005, 0.02]\,\text{s}$, RK4 provides high numerical fidelity with predictable computation per step.

---

## 5. Numerical Stability Considerations
- **Tire Force Damping:** The Magic Formula tire curves produce strong restoring forces and cornering stiffness. At very low speeds ($v_x < 0.05\,\text{m/s}$), slip ratio regularization thresholds ($\epsilon_{\text{slip}} = 0.1\,\text{m/s}$) prevent algebraic singularity and stiffness explosion.
- **Step Size Selection:** For representative parameters ($m = 60\,\text{kg}, I_z = 5.0\,\text{kg}\cdot\text{m}^2$), time steps $\Delta t \leq 0.02\,\text{s}$ maintain stability across cornering, acceleration, and braking maneuvers.

---

## 6. Assumptions & Limitations
1. **Planar Motion:** Roll, pitch, and heave motions are neglected; load transfer is evaluated quasi-statically.
2. **Prescribed Wheel Angular Velocity:** Wheel rotational inertia ($\dot{\omega}$) and motor drive dynamics are not included.
3. **Homogeneous Road Surface:** Per-wheel varying ground friction parameters are deferred to future terrain modeling phases.
4. **Open-Loop Prescribed Controls:** The simulator integrates given steering $\delta(t)$ and wheel speeds $\omega(t)$; closed-loop path following is left for future controller phases.

---

## 7. Verification & Validation
The automated test suite `matlab/simulation/verifyVehicleSimulation.m` validates 25 distinct test scenarios:
- Dimensional and time vector correctness
- Zero-input equilibrium and straight-line motion
- Positive/negative steering response and cornering arcs
- Analytical global position and constant velocity comparisons
- Exact bitwise reproducibility across identical runs
- Boundedness ($\text{no NaN/Inf}$) across dynamic maneuvers
- Convergence rate verification upon step size reduction ($\Delta t \to \Delta t/2$)
- Dynamic callback execution for time-varying steering and throttle functions
- Consistency with underlying Phase 3F closed-loop derivatives
- Long-duration numerical stability and regression compatibility across all previous phases.
