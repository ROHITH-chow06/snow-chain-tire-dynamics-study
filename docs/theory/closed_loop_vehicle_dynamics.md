# Phase 3F: Closed-Loop Four-Wheel Vehicle Dynamics

## 1. Motivation
In Phase 3E, vehicle accelerations $(a_x, a_y)$ were supplied as **explicit external inputs** to the dynamic normal-load model (`dynamicNormalLoadDistribution.m`). This created an open-loop evaluation boundary: a downstream controller or simulation driver was responsible for providing consistent acceleration values.

In a physically self-consistent vehicle model, tire contact forces $(F_x, F_y)$ generate vehicle accelerations $(a_x, a_y)$, which govern dynamic normal loads $F_z$, which in turn govern tire forces:
$$\mathbf{a} \;\longleftrightarrow\; \mathbf{F}_z(\mathbf{a}) \;\longleftrightarrow\; \mathbf{F}_{\text{tire}}(\boldsymbol{\kappa}, \boldsymbol{\alpha}, \mathbf{F}_z) \;\longleftrightarrow\; \mathbf{a}$$

Phase 3F resolves this algebraic coupling using a quasi-static fixed-point iteration, producing self-consistent state derivatives $\dot{\mathbf{x}} = [\dot{v}_x; \dot{v}_y; \dot{r}]^T$ suitable for ODE-based time integration in Phase 3G.

> [!IMPORTANT]
> Phase 3F is **NOT** yet a complete time-domain simulation. It is a validated closed-loop force/acceleration derivative framework. Phase 3G will handle numerical time integration and trajectory simulation.

---

## 2. Phase 3E Limitation

In Phase 3E, `dynamicTireForces.m` was called as:
```
[Fx, Fy, Fz] = dynamicTireForces(vx, vy, r, delta, omega, ax, ay, params)
```
The caller supplied `ax, ay` externally, with no enforcement that these values were consistent with the tire forces returned. Phase 3F closes this loop so that the returned forces and accelerations are mutually consistent.

---

## 3. Coordinate Frame & Reference Conventions

| Quantity | Direction |
|:---|:---|
| $+x$ | Forward (vehicle body frame) |
| $+y$ | Left (vehicle body frame) |
| $+z$ | Upward |
| $r > 0$ | Counter-clockwise yaw rate (viewed from above) |
| $F_z > 0$ | Upward ground reaction force |

---

## 4. Body Acceleration Definitions: Derivation

Let the vehicle body-fixed frame $\mathcal{B}$ rotate with yaw rate $\vec{\boldsymbol{\omega}} = r\hat{\mathbf{k}}$. The absolute (inertial) translational acceleration of the CG, resolved in body frame, is:
$$\vec{\mathbf{a}}_{\text{CG}} = \left(\frac{d\vec{\mathbf{v}}_{\text{CG}}}{dt}\right)_{\text{inertial}} = \underbrace{(\dot{v}_x - v_y r)}_{\displaystyle a_x} \hat{\mathbf{i}}_B + \underbrace{(\dot{v}_y + v_x r)}_{\displaystyle a_y} \hat{\mathbf{j}}_B$$

The pitch and roll overturning moments about the ground plane are:
$$M_{\text{pitch}} = m a_x h_{\text{CG}} = \left(\sum F_{x,\text{body}}\right) h_{\text{CG}}$$
$$M_{\text{roll}} = m a_y h_{\text{CG}} = \left(\sum F_{y,\text{body}}\right) h_{\text{CG}}$$

> [!IMPORTANT]
> **Acceleration Frame Decision:** The physical accelerations for dynamic load transfer are:
> $$a_x = \frac{\sum F_{x,\text{body}}}{m} = \dot{v}_x - v_y r, \quad a_y = \frac{\sum F_{y,\text{body}}}{m} = \dot{v}_y + v_x r$$
>
> Feeding state derivatives $(\dot{v}_x, \dot{v}_y)$ directly would be **physically erroneous** during steady cornering, where $\dot{v}_y = 0$ but centripetal acceleration $a_y = v_x r \neq 0$ produces real lateral load transfer.

---

## 5. Newton-Euler Planar Equations of Motion

The three-DOF planar equations for a rigid vehicle body are:
$$\sum F_{x,\text{body}} = m (\dot{v}_x - v_y r) \implies \dot{v}_x = \frac{F_{x,\text{total}}}{m} + v_y r = a_x + v_y r$$
$$\sum F_{y,\text{body}} = m (\dot{v}_y + v_x r) \implies \dot{v}_y = \frac{F_{y,\text{total}}}{m} - v_x r = a_y - v_x r$$
$$\sum M_z = I_z \dot{r} \implies \dot{r} = \frac{M_{z,\text{total}}}{I_z}$$

---

## 6. Wheel Kinematics (Phase 3A)

For each wheel $i \in \{\text{FL, FR, RL, RR}\}$, rigid-body velocity components at the contact point:
$$v_{x,\text{body},i} = v_x - r y_i, \quad v_{y,\text{body},i} = v_y + r x_i$$

Transformed into the wheel frame by rotation through steering angle $\delta_i$:
$$v_{x,\text{wheel},i} = \cos\delta_i \cdot v_{x,\text{body},i} + \sin\delta_i \cdot v_{y,\text{body},i}$$
$$v_{y,\text{wheel},i} = -\sin\delta_i \cdot v_{x,\text{body},i} + \cos\delta_i \cdot v_{y,\text{body},i}$$

Longitudinal slip ratio (Phase 1A):
$$\kappa_i = \frac{R\omega_i - v_{x,\text{wheel},i}}{\max(|v_{x,\text{wheel},i}|, \epsilon)}$$

Slip angle (Phase 3A):
$$\alpha_i = \text{atan2}(v_{y,\text{wheel},i}, v_{x,\text{wheel},i})$$

> [!NOTE]
> Wheel kinematics $(\boldsymbol{\kappa}, \boldsymbol{\alpha})$ depend only on state $(v_x, v_y, r)$ and prescribed controls $(\delta, \boldsymbol{\omega})$. They are **constant during the algebraic force-balance iteration** and computed only once per derivative evaluation.

---

## 7. Dynamic Normal Load Transfer (Phase 3D)

Individual wheel vertical loads from quasi-static load transfer:
$$F_{z,\text{FL}} = \frac{m g l_r}{2L} - \frac{m a_x h_{\text{CG}}}{2L} - \frac{l_r}{L} \frac{m a_y h_{\text{CG}}}{t_f}$$
$$F_{z,\text{FR}} = \frac{m g l_r}{2L} - \frac{m a_x h_{\text{CG}}}{2L} + \frac{l_r}{L} \frac{m a_y h_{\text{CG}}}{t_f}$$
$$F_{z,\text{RL}} = \frac{m g l_f}{2L} + \frac{m a_x h_{\text{CG}}}{2L} - \frac{l_f}{L} \frac{m a_y h_{\text{CG}}}{t_r}$$
$$F_{z,\text{RR}} = \frac{m g l_f}{2L} + \frac{m a_x h_{\text{CG}}}{2L} + \frac{l_f}{L} \frac{m a_y h_{\text{CG}}}{t_r}$$
Conservation: $\sum F_{z,i} = m g$ for all valid $(a_x, a_y)$.

---

## 8. Combined-Slip Tire Forces (Phases 3B & 3C)

Reduced PAC2002-style weighting:
$$F_{x,i} = G_{x\alpha}(\alpha_i) \cdot F_{x0}(\kappa_i, F_{z,i}), \quad G_{x\alpha}(\alpha) = \frac{\cos(C_{x\alpha}\arctan(B_{x\alpha}\alpha_s - E_{x\alpha}(B_{x\alpha}\alpha_s - \arctan(B_{x\alpha}\alpha_s))))}{G_0}$$
$$F_{y,i} = G_{y\kappa}(\kappa_i) \cdot F_{y0}(\alpha_i, F_{z,i}), \quad G_{y\kappa}(\kappa) = \frac{\cos(C_{y\kappa}\arctan(B_{y\kappa}\kappa_s - E_{y\kappa}(B_{y\kappa}\kappa_s - \arctan(B_{y\kappa}\kappa_s))))}{G_0}$$

---

## 9. Body Frame Force Transformation (Phase 3A)

From wheel frame to vehicle body frame:
$$F_{x,\text{body},i} = \cos\delta_i F_{x,i} - \sin\delta_i F_{y,i}$$
$$F_{y,\text{body},i} = \sin\delta_i F_{x,i} + \cos\delta_i F_{y,i}$$

Total body forces and yaw moment:
$$F_{x,\text{total}} = \sum_{i} F_{x,\text{body},i}, \quad F_{y,\text{total}} = \sum_{i} F_{y,\text{body},i}$$
$$M_{z,\text{total}} = \sum_{i} (x_i F_{y,\text{body},i} - y_i F_{x,\text{body},i})$$

---

## 10. Algebraic Coupling & Fixed-Point Formulation

The closed-loop evaluation problem is: find $(a_x^*, a_y^*)$ satisfying:
$$a_x^* = \frac{F_{x,\text{total}}(\mathbf{F}_z(a_x^*, a_y^*))}{m}, \quad a_y^* = \frac{F_{y,\text{total}}(\mathbf{F}_z(a_x^*, a_y^*))}{m}$$

This is a fixed-point equation $\mathbf{a}^* = \mathbf{T}(\mathbf{a}^*)$ solved by Picard (successive substitution) iteration.

---

## 11. Fixed-Point Solution Algorithm

**Inputs:** state $\mathbf{x} = [v_x; v_y; r]$, controls $(\delta, \boldsymbol{\omega})$, solver options.

**Step 0:** Compute $(\boldsymbol{\kappa}, \boldsymbol{\alpha})$ via `wheelKinematics.m` — held constant throughout.

**Iteration:** For $k = 0, 1, 2, \ldots$:
1. $\mathbf{F}_z^{(k)} = \text{dynamicNormalLoadDistribution}(\text{params}, a_x^{(k)}, a_y^{(k)})$
2. $[\mathbf{F}_x^{(k)}, \mathbf{F}_y^{(k)}] = \text{combinedSlipTireForce}(\boldsymbol{\kappa}, \boldsymbol{\alpha}, \mathbf{F}_z^{(k)})$
3. Rotate forces to body frame; compute $F_{x,\text{total}}^{(k)}, F_{y,\text{total}}^{(k)}$
4. $a_{x,\text{eval}} = F_{x,\text{total}}^{(k)} / m$, $a_{y,\text{eval}} = F_{y,\text{total}}^{(k)} / m$
5. $\varepsilon^{(k)} = \|[a_{x,\text{eval}} - a_x^{(k)},\; a_{y,\text{eval}} - a_y^{(k)}]\|_\infty$
6. If $\varepsilon^{(k)} < \epsilon_{\text{tol}}$: **converged**, set $a_x^* = a_{x,\text{eval}}$, $a_y^* = a_{y,\text{eval}}$, stop.
7. Update: $a_x^{(k+1)} = (1-\gamma)a_x^{(k)} + \gamma a_{x,\text{eval}}$, $a_y^{(k+1)} = (1-\gamma)a_y^{(k)} + \gamma a_{y,\text{eval}}$

**Default parameters:** $\epsilon_{\text{tol}} = 10^{-6}\,\text{m/s}^2$, $\text{max\_iter} = 50$, $a^{(0)} = [0; 0]$, $\gamma = 1.0$.

> [!NOTE]
> The load-transfer coupling is expected to be weak for the representative vehicle parameters, but convergence is not assumed a priori. The solver explicitly monitors convergence error and maximum iteration count. Verification characterizes actual convergence behavior.

---

## 12. Convergence Criterion & Termination

**Convergence:** $\varepsilon^{(k)} < \epsilon_{\text{tol}}$ (default $10^{-6}\,\text{m/s}^2$).

**Failure:** If $k > \text{max\_iter}$ without convergence, the solver throws: `solveQuasiStaticForceBalance:maxIterationsExceeded`.

**Final consistency:** After convergence, forces $\mathbf{F}_x, \mathbf{F}_y, \mathbf{F}_z$ are re-evaluated at the final $(a_x^*, a_y^*)$ to ensure all returned quantities are mutually consistent at the same acceleration level.

---

## 13. Wheel-Lift Behavior

If any acceleration iterate causes $F_{z,i} \leq 0$, `dynamicNormalLoadDistribution.m` throws `dynamicNormalLoadDistribution:negativeNormalLoad`. This error propagates cleanly through `solveQuasiStaticForceBalance` and `closedLoopVehicleDerivative` without silent clamping. Calling code receives the diagnostic error.

---

## 14. Assumptions & Limitations

- **Quasi-static load transfer:** Suspension dynamics, roll-rate, pitch-rate, wheel hop, and sprung mass oscillations are not modeled.
- **Planar 3-DOF body:** No vertical, pitch, or roll state variables are integrated.
- **Prescribed wheel speeds:** Wheel rotational dynamics (motor torque, brake torque, wheel inertia) are not modeled.
- **Reduced combined-slip model:** Representative PAC2002-style weighting coefficients; not experimentally identified from a physical rover tire.
- **Symmetric road surface:** Per-wheel asymmetric friction conditions are not yet implemented.
- **Front-axle steering only:** $\delta_{\text{RL}} = \delta_{\text{RR}} = 0$.

---

## 15. Numerical Considerations

- Kinematics are computed once per derivative call, not re-evaluated during the algebraic loop.
- Relaxation factor $\gamma < 1$ is available for scenarios where the default full-step update is insufficiently damped.
- Convergence observed in 2–15 iterations for representative operating conditions verified in the test suite.
- Initial guess $\mathbf{a}^{(0)} = [0; 0]$ is adequate for mild maneuvers; warm-starting from previous timestep accelerations is straightforward to add in Phase 3G.

---

## 16. Verification Results

The automated test suite `verifyClosedLoopVehicleDynamics.m` confirms **22/22 tests PASSED** in MATLAB R2026a, covering: standstill equilibrium, pure traction/braking, zero-force cruise, steering response (both signs), yaw moment coupling, four-wheel load transfer effects, combined-slip force reduction, force/moment balance identities, fixed-point convergence characterization, tight tolerance sensitivity, deterministic max-iteration rejection, wheel-lift error propagation, numerical robustness across operating grids, left/right and front/rear symmetry, exact consistency with Phase 3E at converged accelerations, and complete 3-DOF kinematic/dynamic self-consistency.

---

## 17. Future Extensions (Phase 3G & Beyond)

- **Phase 3G:** Numerical ODE integration of $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$ using `closedLoopVehicleDerivative.m` to produce global trajectory $(X(t), Y(t), \psi(t))$.
- Warm-start initialization from previous timestep accelerations to reduce iteration count during time integration.
- Suspension roll stiffness-based front/rear lateral transfer allocation.
- Per-wheel road surface friction parameterization.
- Wheel rotational dynamics and drivetrain model.
