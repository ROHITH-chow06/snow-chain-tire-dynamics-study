# Pure Lateral Magic Formula Tire Force Model

## 1. Scope and Modeling Objectives

This document formulates the steady-state, pure-lateral tire force model for the `VehicleMobilityResearch` project.

The objective of Phase 3B is to establish a research-grounded constitutive relationship that determines lateral tire cornering force ($F_y$) as a function of the wheel velocity slip angle ($\alpha_v$) and vertical normal load ($F_z$).

### Scope Boundaries:
- **Pure Lateral Cornering Only:** The model evaluates lateral tire force under pure side-slip conditions ($\kappa = 0$).
- **No Combined Slip:** Coupling between longitudinal tractive/braking forces and lateral cornering forces (friction circle / friction ellipse) is excluded.
- **No Dynamic Load Sensitivity:** Model coefficients ($B_y, C_y, D_y, E_y$) are constant with respect to normal load $F_z$.
- **No Camber / Inclination Angle:** The wheel plane is perpendicular to the ground plane ($\gamma = 0$).
- **No Horizontal/Vertical Shifts:** $S_{hy} = 0, S_{vy} = 0$ (conicity/ply-steer effects are omitted).
- **No Aligning Moment / Relaxation:** Pneumatic trail, self-aligning moment ($M_z$), and high-frequency carcass relaxation dynamics are excluded.
- **Environmental Parameters:** The baseline parameters represent a nominal dry surface; separate wet, snow, or ice lateral parameterizations are deferred to subsequent phases.

---

## 2. Research Basis and Mathematical Formulation

### 2.1 Generic Magic Formula Structure
The semi-empirical Pacejka Magic Formula provides a continuous, analytical trigonometric function capable of capturing the linear small-slip regime, the non-linear transition, peak force saturation, and post-peak behavior:

$$y(x) = D \sin\left( C \arctan\left( B x - E \left( B x - \arctan(B x) \right) \right) \right)$$

For pure lateral tire-force generation, the independent variable is the slip angle ($x = \alpha_v$) and the output is the magnitude of the cornering force response ($Y = F_{y,\text{MF}}$).

### 2.2 Project-Specific Lateral Force Equation
In this project, the pure lateral Magic Formula is implemented as:

$$F_{y,\text{MF}} = F_z \cdot D_y \cdot \sin\left( C_y \cdot \arctan\left( B_y \alpha_v - E_y \left( B_y \alpha_v - \arctan(B_y \alpha_v) \right) \right) \right)$$

Applying the project's restoring force sign convention:

$$F_y = -F_{y,\text{MF}}$$

where:
- $\alpha_v$: Signed wheel velocity slip angle $[\text{rad}]$.
- $F_z$: Vertical tire normal load $[\text{N}]$ ($F_z > 0$).
- $B_y$: Lateral stiffness factor $[-]$.
- $C_y$: Lateral shape factor $[-]$.
- $D_y$: Lateral peak friction factor $[-]$.
- $E_y$: Lateral curvature factor $[-]$.
- $F_y$: Generated lateral force in the wheel coordinate frame $[\text{N}]$.

---

## 3. Slip-Angle and Force Sign Convention

### 3.1 Kinematic Velocity Slip Angle ($\alpha_v$)
As formulated in Phase 3A ([wheelSlipAngle.m](file:///c:/MATLAB/VehicleMobilityResearch/matlab/tire/wheelSlipAngle.m)), the wheel velocity angle is explicitly defined in the wheel coordinate frame ($+x$ forward, $+y$ leftward) by:

$$\alpha_v = \text{atan2}\left(v_{y,\text{wheel}}, v_{x,\text{wheel}}\right)$$

### 3.2 Lateral Restoring Force Direction
Under physical tire-road interaction, when a rolling tire experiences a lateral velocity component to the left ($v_{y,\text{wheel}} > 0 \implies \alpha_v > 0$), the tire contact patch generates a restoring friction force directed to the right ($F_y < 0$) to oppose lateral motion.

Conversely, when the tire experiences a lateral velocity component to the right ($v_{y,\text{wheel}} < 0 \implies \alpha_v < 0$), the contact patch generates a restoring friction force directed to the left ($F_y > 0$).

Therefore, the relationship between lateral tire force and velocity slip angle is:
- **$\alpha_v = 0$:** Zero lateral velocity $\implies F_y = 0$.
- **$\alpha_v > 0$:** Positive leftward velocity angle $\implies F_y < 0$ (rightward force).
- **$\alpha_v < 0$:** Negative rightward velocity angle $\implies F_y > 0$ (leftward force).
- **Odd Symmetry:** $F_y(-\alpha_v) = -F_y(\alpha_v)$.

---

## 4. Parameter Roles and Cornering Stiffness

| Parameter | Symbol | Role and Interpretation | SI Dimension |
| :--- | :--- | :--- | :--- |
| Stiffness Factor | $B_y$ | Controls initial slope when combined with $C_y, D_y$ | Dimensionless |
| Shape Factor | $C_y$ | Governs curve profile and asymptotic limits | Dimensionless |
| Peak Factor | $D_y$ | Normalized peak lateral friction coefficient ($\mu_{y,\text{peak}} = D_y$) | Dimensionless |
| Curvature Factor | $E_y$ | Controls transition curvature near and beyond peak | Dimensionless |

### Initial Cornering Stiffness ($K_\alpha$)
The small-angle linearization around zero slip ($\alpha_v \approx 0$) gives:

$$\left.\frac{\partial F_y}{\partial \alpha_v}\right|_{\alpha_v = 0} = -B_y \cdot C_y \cdot D_y \cdot F_z = -K_\alpha$$

where the positive initial cornering stiffness is:

$$K_\alpha = B_y \cdot C_y \cdot D_y \cdot F_z \quad [\text{N/rad}]$$

Thus, in the linear regime ($|\alpha_v| \le 0.03\,\text{rad} \approx 1.7^\circ$):
$$F_y \approx -K_\alpha \cdot \alpha_v$$

---

## 5. Representative Baseline Parameter Set

The following representative parameters are selected for model verification and baseline characterization:

| Parameter | Symbol | Value | Classification / Role |
| :--- | :--- | :--- | :--- |
| Vertical Normal Load | $F_z$ | $4000.0\,\text{N}$ | Reference normal load |
| Lateral Stiffness Factor | $B_y$ | $10.0$ | Baseline stiffness factor |
| Lateral Shape Factor | $C_y$ | $1.30$ | Classical lateral shape factor |
| Lateral Peak Factor | $D_y$ | $1.00$ | Normalized peak lateral friction ($\mu_{y,\text{peak}} = 1.0$) |
| Lateral Curvature Factor | $E_y$ | $-1.00$ | Curvature parameter ensuring smooth saturation |

> **Research Integrity Notice:** These parameters are representative values selected for model development and verification. They do **not** represent empirical measurements of any specific commercial tire.

### Resulting Baseline Derived Quantities:
- **Normalized Cornering Stiffness:** $K_\alpha / F_z = B_y \cdot C_y \cdot D_y = 13.0\,\text{rad}^{-1} \approx 0.2269\,\text{deg}^{-1}$.
- **Absolute Cornering Stiffness ($F_z = 4000\,\text{N}$):** $K_\alpha = 52000.0\,\text{N/rad} \approx 907.6\,\text{N/deg}$.
- **Absolute Cornering Stiffness ($F_z = 147.15\,\text{N}$, Rover wheel):** $K_\alpha = 1912.95\,\text{N/rad} \approx 33.39\,\text{N/deg}$.
- **Peak Lateral Force ($F_z = 4000\,\text{N}$):** $|F_{y,\text{peak}}| = D_y \cdot F_z = 4000.00\,\text{N}$.
- **Peak Slip Angle:** $|\alpha_{\text{peak}}| \approx 0.1860\,\text{rad} \approx 10.66^\circ$.

---

## 6. Numerical Verification Results

The implementation in [matlab/tire/magicFormulaLateral.m](file:///c:/MATLAB/VehicleMobilityResearch/matlab/tire/magicFormulaLateral.m) was verified using the automated test suite [matlab/simulation/verifyMagicFormulaLateral.m](file:///c:/MATLAB/VehicleMobilityResearch/matlab/simulation/verifyMagicFormulaLateral.m) executed in **MATLAB R2026a**.

### Verification Summary:

| Test Case | Description | Theoretical Expectation | Observed MATLAB Output | Status |
| :--- | :--- | :--- | :--- | :--- |
| **Test 1** | Zero slip angle ($\alpha = 0$) | $F_y = 0.0\,\text{N}$ | $F_y = 0.000000\,\text{N}$ | **PASSED** |
| **Test 2** | Positive velocity angle ($\alpha = +0.10\,\text{rad}$) | $F_y < 0$ (restoring force) | $F_y = -3645.48\,\text{N}$ | **PASSED** |
| **Test 3** | Negative velocity angle ($\alpha = -0.10\,\text{rad}$) | $F_y > 0$ (restoring force) | $F_y = +3645.48\,\text{N}$ | **PASSED** |
| **Test 4** | Odd symmetry check | $F_y(-\alpha) = -F_y(\alpha)$ | Exact anti-symmetry verified ($< 10^{-12}\,\text{N}$) | **PASSED** |
| **Test 5** | Vectorized evaluation | Array output matches scalar loop | $201$ points matched ($< 10^{-12}\,\text{N}$) | **PASSED** |
| **Test 6** | Normal-load scaling ($2 \times F_z$) | $F_y(2F_z) / F_y(F_z) = 2.0$ | Exact linear scaling ($2.0000$) | **PASSED** |
| **Test 7** | Invalid normal load ($F_z \le 0$) | Error rejection for $F_z \le 0$ | $F_z = 0$ and $F_z < 0$ successfully rejected | **PASSED** |
| **Test 8** | Dimension mismatch rejection | Array dimension mismatch rejection | Error ID `magicFormulaLateral:dimensionMismatch` | **PASSED** |
| **Test 9** | Small-slip initial slope ($dF_y/d\alpha$) | $\left.\frac{dF_y}{d\alpha}\right|_0 = -52000.0\,\text{N/rad}$ | Analytical $-52000.0$, numerical $-52000.0$ (rel err $2.82 \times 10^{-7}\%$) | **PASSED** |
| **Test 10** | Peak-force sanity across sweep | Finite peak $|F_{y,\text{max}}| = 4000.0\,\text{N}$ | $|F_y| = 4000.00\,\text{N}$ at $\alpha = \pm 0.1860\,\text{rad}$ ($\pm 10.66^\circ$) | **PASSED** |

---

## 7. Model Assumptions and Limitations

1. **Quasi-Steady Pure Lateral Slip:** Dynamic transient carcass buildup (relaxation length $\sigma_y$) is omitted.
2. **Constant Parameter Formulation:** $B_y, C_y, D_y, E_y$ do not vary non-linearly with normal load $F_z$.
3. **No Camber Thrust:** Vehicle roll and wheel camber inclination ($\gamma$) are set to zero.
4. **No Combined Slip:** Independent evaluation only ($\kappa = 0$); interaction with simultaneous longitudinal traction/braking is not included in Phase 3B.
5. **No Self-Aligning Moment:** Contact patch pneumatic trail and resulting $M_z$ moments about the wheel vertical axis are omitted.

---

## 8. Documented Future Extensions

1. **Environmental Lateral Parameterization:** Formulation of distinct dry, wet, snow, and ice parameter sets for $B_y, C_y, D_y, E_y$.
2. **Combined Slip Formulation:** 2D friction ellipse / circle or Pacejka weighting functions $G_{xa}(\alpha, \kappa)$ and $G_{yk}(\alpha, \kappa)$.
3. **Load-Dependent Parameter Scaling:** Multi-load parameterization capturing cornering stiffness de-rating under heavy vertical loads.
4. **Self-Aligning Moment Model ($M_z$):** Computation of pneumatic trail ($t$) and residual moment ($M_{zr}$) for steering torque feedback.
5. **Integration with Four-Wheel Dynamics (Phase 3C/4):** Direct coupling of `magicFormulaLongitudinal.m` and `magicFormulaLateral.m` into `fourWheelDynamics.m` during vehicle maneuver simulations.

---

## 9. References

1. **Pacejka, H. B.** (2012). *Tire and Vehicle Dynamics* (3rd ed.). Butterworth-Heinemann / Elsevier. (Chapter 1: "Tire Characteristics and Vehicle Handling and Stability", Chapter 4: "Semi-Empirical Tyre Models").
2. **Gillespie, T. D.** (1992). *Fundamentals of Vehicle Dynamics*. Society of Automotive Engineers (SAE), Warrendale, PA. (Chapter 6: "Cornering", Chapter 10: "Tires").
3. **Rajamani, R.** (2012). *Vehicle Dynamics and Control* (2nd ed.). Springer US. (Chapter 2: "Lateral Vehicle Dynamics", Chapter 3: "Tire Models").
4. **SAE International.** (2022). *Vehicle Dynamics Terminology* (SAE Recommended Practice J670_202206). SAE International.
