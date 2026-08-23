# Phase 3A — Four-Wheel Rover Kinematics and 3-DOF Vehicle Dynamics Framework

## 1. Objective and Scope

Phase 3A establishes the four-wheel vehicle dynamics and planar kinematics framework for the `VehicleMobilityResearch` project.

The primary objective is to develop a modular, research-grade four-wheel rigid-body simulation framework that preserves individual wheel states ($\text{FL}, \text{FR}, \text{RL}, \text{RR}$) rather than collapsing into a lumped single-track (bicycle) approximation. This framework provides the dynamic infrastructure that will subsequently interface with tire constitutive force models (Phase 3B) and terrain-dependent traction studies.

### Scope Boundaries:
- **Tire Forces:** External/test inputs to the framework. No lateral Magic Formula (`magicFormulaLateral.m`) is implemented in Phase 3A.
- **Combined Slip:** No combined longitudinal/lateral friction ellipse or friction circle.
- **Load Transfer:** Static vertical load distribution only. Dynamic pitch/roll load transfers are excluded.
- **Wheel Dynamics:** Kinematic slip calculations only. Wheel rotational inertias and drivetrain torques are excluded.
- **Road Conditions:** Uniform road condition across all wheels; asymmetric conditions are documented for future extension.

---

## 2. Framework Architecture and Module Structure

The Phase 3A implementation is organized into modular functions within `matlab/vehicle/` and verification suites in `matlab/simulation/`:

```
VehicleMobilityResearch/
├── matlab/
│   ├── tire/
│   │   ├── longitudinalSlipRatio.m       [Phase 1A - Unmodified]
│   │   └── magicFormulaLongitudinal.m    [Phase 1B - Unmodified]
│   ├── vehicle/
│   │   ├── roverParameters.m             [Phase 3A - Representative rover geometry & mass]
│   │   ├── wheelSlipAngle.m              [Phase 3A - Signed wheel velocity slip angle]
│   │   ├── wheelKinematics.m             [Phase 3A - 4-wheel planar kinematics & slip]
│   │   ├── normalLoadDistribution.m      [Phase 3A - Static per-wheel normal loads]
│   │   └── fourWheelDynamics.m           [Phase 3A - 3-DOF planar equations of motion]
│   └── simulation/
│       ├── verifyWheelKinematics.m        [Phase 3A - Kinematics verification suite]
│       └── verifyFourWheelDynamics.m      [Phase 3A - Dynamics verification suite]
└── docs/
    ├── theory/
    │   └── four_wheel_rover_dynamics.md  [Phase 3A - Mathematical formulation]
    └── methodology/
        └── phase3a_vehicle_model.md      [Phase 3A - Methodology & verification record]
```

---

## 3. Representative Rover Parameters

To provide transparent, physically grounded baseline values for simulation, a representative small four-wheel research rover geometry is established.

> **Engineering Notice:** These values represent an idealized research vehicle configuration and do not claim to match measured data of any commercial rover.

| Parameter | Symbol | Value | SI Unit | Engineering Rationale |
| :--- | :--- | :--- | :--- | :--- |
| Vehicle Total Mass | $m$ | $60.0$ | $\text{kg}$ | Representative small autonomous ground rover |
| CG to Front Axle | $l_f$ | $0.40$ | $\text{m}$ | Symmetric longitudinal weight distribution |
| CG to Rear Axle | $l_r$ | $0.40$ | $\text{m}$ | Total wheelbase $L = l_f + l_r = 0.80\,\text{m}$ |
| Front Track Width | $t_f$ | $0.60$ | $\text{m}$ | Representative lateral track width |
| Rear Track Width | $t_r$ | $0.60$ | $\text{m}$ | Equal front and rear track |
| Wheel Rolling Radius | $R$ | $0.15$ | $\text{m}$ | Rover pneumatic / solid elastomeric tire |
| Gravitational Acceleration | $g$ | $9.81$ | $\text{m/s}^2$ | Standard terrestrial gravity |
| Yaw Moment of Inertia | $I_z$ | $5.00$ | $\text{kg}\cdot\text{m}^2$ | Derived via simplified rectangular mass distribution |

### Simplified Yaw Moment of Inertia Approximation
The yaw moment of inertia $I_z$ about the vertical $z$-axis through the CG is derived from a homogeneous rectangular mass approximation of length $L = l_f + l_r = 0.80\,\text{m}$ and width $W = t_f = 0.60\,\text{m}$:

$$I_z = \frac{m}{12}\left(L^2 + W^2\right) = \frac{60.0}{12}\left(0.80^2 + 0.60^2\right) = 5.0 \cdot (0.64 + 0.36) = 5.00\,\text{kg}\cdot\text{m}^2$$

---

## 4. Kinematics and Slip Formulations

### 4.1 Wheel Center Velocity Vectors
For each wheel location $i \in \{\text{FL}, \text{FR}, \text{RL}, \text{RR}\}$ with position $\boldsymbol{r}_i = [x_i, y_i, 0]^T$ relative to the CG:

$$\begin{bmatrix} v_{x,\text{body},i} \\ v_{y,\text{body},i} \end{bmatrix} = \begin{bmatrix} v_x - r y_i \\ v_y + r x_i \end{bmatrix}$$

Rotating by the steer angle $\delta_i$ into the wheel frame:
$$\begin{bmatrix} v_{x,\text{wheel},i} \\ v_{y,\text{wheel},i} \end{bmatrix} = \begin{bmatrix} \cos\delta_i & \sin\delta_i \\ -\sin\delta_i & \cos\delta_i \end{bmatrix} \begin{bmatrix} v_{x,\text{body},i} \\ v_{y,\text{body},i} \end{bmatrix}$$

### 4.2 Longitudinal Slip Ratio ($\kappa_i$)
Evaluated using the existing validated Phase 1A function [longitudinalSlipRatio.m](file:///c:/MATLAB/VehicleMobilityResearch/matlab/tire/longitudinalSlipRatio.m):
$$\kappa_i = \frac{R \omega_i - v_{x,\text{wheel},i}}{\max\left(|v_{x,\text{wheel},i}|, \epsilon_{\text{slip}}\right)}$$

### 4.3 Wheel Velocity Slip Angle ($\alpha_i$)
Evaluated using [wheelSlipAngle.m](file:///c:/MATLAB/VehicleMobilityResearch/matlab/vehicle/wheelSlipAngle.m):
$$\alpha_i = \begin{cases} 
\text{atan2}\left(v_{y,\text{wheel},i}, v_{x,\text{wheel},i}\right), & \text{if } \sqrt{v_{x,\text{wheel},i}^2 + v_{y,\text{wheel},i}^2} \ge v_{\text{threshold}} \\
0.0, & \text{if } \sqrt{v_{x,\text{wheel},i}^2 + v_{y,\text{wheel},i}^2} < v_{\text{threshold}}
\end{cases}$$
where $v_{\text{threshold}} = 10^{-4}\,\text{m/s}$.

---

## 5. Static Normal Load Distribution

Under static gravitational equilibrium:
$$F_{z,\text{total}} = m g = 60.0 \times 9.81 = 588.60\,\text{N}$$

Axle loads:
$$F_{z,\text{front}} = F_{z,\text{total}} \left(\frac{l_r}{l_f + l_r}\right) = 294.30\,\text{N}, \quad F_{z,\text{rear}} = F_{z,\text{total}} \left(\frac{l_f}{l_f + l_r}\right) = 294.30\,\text{N}$$

For the Phase 3A baseline equilibrium case, equal left/right static normal-load distribution is assumed:
$$F_{z,\text{FL}} = F_{z,\text{FR}} = 147.15\,\text{N}, \quad F_{z,\text{RL}} = F_{z,\text{RR}} = 147.15\,\text{N}$$

> **Model Architecture Note:**
> For the Phase 3A baseline equilibrium case, equal left/right static normal-load distribution is assumed because the vehicle CG is laterally centered and dynamic load transfer is not yet modeled. Individual wheel kinematics, slip states, forces ($F_x, F_y$), and yaw moments remain independently represented for all four wheels ($\text{FL}, \text{FR}, \text{RL}, \text{RR}$), preserving the architecture required for future asymmetric loading and terrain conditions without reducing the system to a bicycle model.

---

## 6. 3-DOF Planar Rigid-Body Dynamics

The 3-DOF planar equations of motion implemented in [fourWheelDynamics.m](file:///c:/MATLAB/VehicleMobilityResearch/matlab/vehicle/fourWheelDynamics.m) are:

$$\begin{aligned}
\dot{v}_x &= \frac{1}{m}\sum_{i=1}^4 F_{x,\text{body},i} + v_y r \\
\dot{v}_y &= \frac{1}{m}\sum_{i=1}^4 F_{y,\text{body},i} - v_x r \\
\dot{r}   &= \frac{1}{I_z}\sum_{i=1}^4 \left( x_i F_{y,\text{body},i} - y_i F_{x,\text{body},i} \right)
\end{aligned}$$

where the wheel-to-body force transformation is:
$$\begin{aligned}
F_{x,\text{body},i} &= \cos\delta_i \cdot F_{x,\text{wheel},i} - \sin\delta_i \cdot F_{y,\text{wheel},i} \\
F_{y,\text{body},i} &= \sin\delta_i \cdot F_{x,\text{wheel},i} + \cos\delta_i \cdot F_{y,\text{wheel},i}
\end{aligned}$$

---

## 7. Numerical Verification Results

Verification scripts were executed in **MATLAB R2026a** on the active platform.

### 7.1 Kinematics Verification Suite (`verifyWheelKinematics.m`)

| Test Case | Description / Operating Point | Analytical Expectation | Observed MATLAB Output | Status |
| :--- | :--- | :--- | :--- | :--- |
| **Test 1** | Standstill: $v_x = v_y = r = 0$, $\omega_i = 0$ | $v_{\text{body}} = 0, v_{\text{wheel}} = 0, \kappa = 0, \alpha = 0$ | All zeros, finite, no $\text{NaN}/\text{Inf}$ | **PASSED** |
| **Test 2** | Pure rolling: $v_x = 2.0\,\text{m/s}, \omega_i = 13.333\,\text{rad/s}$<br>Driving slip: $\omega_i = 16.0\,\text{rad/s}$ | $\kappa_{\text{roll}} = 0.0, \alpha = 0.0$<br>$\kappa_{\text{drive}} = (2.4-2.0)/2.0 = +0.2000$ | $v_{x,i} = 2.000\,\text{m/s}, \kappa = 0.0000$<br>$\kappa = +0.2000$ | **PASSED** |
| **Test 3** | Pure yaw: $v_x = v_y = 0, r = +1.0\,\text{rad/s}$ | Left $v_x = -0.30\,\text{m/s}$, Right $v_x = +0.30\,\text{m/s}$<br>Front $v_y = +0.40\,\text{m/s}$, Rear $v_y = -0.40\,\text{m/s}$ | Exact analytical match for all 4 wheels | **PASSED** |
| **Test 4** | Steered forward: $\delta = +0.10\,\text{rad}, v_x = 2.0\,\text{m/s}$ | Front: $v_{x,w} = 2\cos(0.1), v_{y,w} = -2\sin(0.1)$<br>Front $\alpha = -0.1000\,\text{rad}$, Rear $\alpha = 0.0$ | Exact trigonometric match; $\alpha_{\text{front}} = -0.1000\,\text{rad}$ | **PASSED** |
| **Test 5** | Dimensional consistency & ordering | All $4 \times 1$ vectors with $[\text{FL}, \text{FR}, \text{RL}, \text{RR}]$ | Dimensions $[4 \times 1]$, correct ordering | **PASSED** |
| **Test 6** | `wheelSlipAngle` unit tests | Zero at rest; Q1 $\alpha > 0$; Q4 $\alpha < 0$; input validation | Regularization, vectorization, error rejection verified | **PASSED** |

### 7.2 Dynamics Verification Suite (`verifyFourWheelDynamics.m`)

| Test Case | Description / Operating Point | Analytical Expectation | Observed MATLAB Output | Status |
| :--- | :--- | :--- | :--- | :--- |
| **Test 1** | Static load distribution | $\sum F_{z,i} = 588.60\,\text{N}$, $F_{z,\text{front}} = F_{z,\text{rear}} = 294.30\,\text{N}$ | $\sum F_z = 588.60\,\text{N}$, per wheel $= 147.15\,\text{N}$ | **PASSED** |
| **Test 2** | Force transformation: $\delta = 0.15\,\text{rad}, F_{x,w} = 100, F_{y,w} = 20$ | Front $F_{x,b} = 95.8883\,\text{N}, F_{y,b} = 34.7192\,\text{N}$ | Exact trigonometric match | **PASSED** |
| **Test 3** | Synthetic yaw moment sanity check | $M_{z,i} = x_i F_{y,b,i} - y_i F_{x,b,i} = -11.0\,\text{N}\cdot\text{m}$ each<br>$M_{z,\text{total}} = -44.0\,\text{N}\cdot\text{m}$ | Exact analytical moment match ($M_z = -44.0\,\text{N}\cdot\text{m}$) | **PASSED** |
| **Test 4** | Pure symmetric traction: $F_{x,w} = 50\,\text{N}$ each ($200\,\text{N}$ total) | $\dot{v}_x = 200/60 = 3.3333\,\text{m/s}^2, \dot{v}_y = 0, \dot{r} = 0$ | $\dot{v}_x = 3.3333\,\text{m/s}^2, \dot{v}_y = 0, \dot{r} = 0$ | **PASSED** |
| **Test 5** | Differential traction: Left $-50\,\text{N}$, Right $+50\,\text{N}$ | $F_{x,\text{tot}} = 0, M_z = +60.0\,\text{N}\cdot\text{m}, \dot{r} = +12.0\,\text{rad/s}^2$ | $M_z = +60.0\,\text{N}\cdot\text{m}, \dot{r} = +12.00\,\text{rad/s}^2$ | **PASSED** |
| **Test 6** | Kinematic coupling: $v_x = 5.0, v_y = 1.0, r = 2.0$, zero forces | $\dot{v}_x = +2.0\,\text{m/s}^2, \dot{v}_y = -10.0\,\text{m/s}^2, \dot{r} = 0$ | $\dot{v}_x = +2.00\,\text{m/s}^2, \dot{v}_y = -10.00\,\text{m/s}^2$ | **PASSED** |
| **Test 7** | Standstill equilibrium: $v_x=v_y=r=0$, zero forces | $\dot{v}_x = 0, \dot{v}_y = 0, \dot{r} = 0$ | $\dot{v}_x = 0, \dot{v}_y = 0, \dot{r} = 0$ | **PASSED** |
| **Test 8** | Straight-line state: $v_x = 3.0, v_y = 0, r = 0$, zero forces | $\dot{v}_x = 0, \dot{v}_y = 0, \dot{r} = 0$ | $\dot{v}_x = 0, \dot{v}_y = 0, \dot{r} = 0$ | **PASSED** |

---

## 8. Documented Future Extensions

The four-wheel architecture implemented in Phase 3A was specifically structured to accommodate future research extensions without requiring architectural restructuring:

1. **Lateral Tire-Force Model (Phase 3B):**
   Formulation of pure lateral Magic Formula ($F_y = f(\alpha, F_z)$) parameterized across dry, wet, and snow surfaces.
2. **Combined Slip Formulation:**
   Implementation of coupled longitudinal/lateral force saturation (friction ellipse or 2D Pacejka combined slip).
3. **Dynamic Load Transfer:**
   Computation of normal load redistributions under longitudinal acceleration ($\dot{v}_x$) and lateral cornering acceleration ($\dot{v}_y$).
4. **Wheel Rotational Dynamics:**
   Integration of wheel spindle equation of motion ($I_w \dot{\omega}_i = T_{\text{drive},i} - T_{\text{brake},i} - R F_{x,i}$).
5. **Ackermann Steering Geometry:**
   Kinematic steering linkage adjustment introducing distinct inner and outer wheel steer angles ($\delta_{\text{in}} > \delta_{\text{out}}$).
6. **Spatially Asymmetric Road Conditions:**
   Per-wheel road-condition assignment (e.g., $\text{FL} = \text{Dry}, \text{FR} = \text{Wet}, \text{RL} = \text{Snow}, \text{RR} = \text{Snow}$) to investigate split-$\mu$ traction and induced yaw moments.
7. **High-Altitude Cold-Terrain / Ladakh Terrain Characterization:**
   Terramechanics modeling for loose aggregate, gravel, and compacted snow/ice.
8. **ROS 2 / Gazebo Integration:**
   Hardware-in-the-loop and physical simulation interface for autonomous navigation algorithms.

---

## 9. References

1. **SAE International.** (2022). *Vehicle Dynamics Terminology* (SAE Recommended Practice J670_202206). SAE International.
2. **Li, B., Raksincharoensak, H., & Wang, R.** (2022). Estimation of Longitudinal Force, Sideslip Angle and Yaw Rate for Four-Wheel Independent Actuated Autonomous Vehicles Based on PWA Tire Model. *Sensors*, 22(3), 885.
3. **Gillespie, T. D.** (1992). *Fundamentals of Vehicle Dynamics*. Society of Automotive Engineers, Warrendale, PA.
4. **Rajamani, R.** (2012). *Vehicle Dynamics and Control* (2nd ed.). Springer US.
