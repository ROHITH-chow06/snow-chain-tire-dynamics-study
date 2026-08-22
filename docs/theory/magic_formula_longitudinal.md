# Pure Longitudinal Tire Force Model (Magic Formula)

## 1. Objective

The objective of Phase 1B is to establish a transparent, baseline semi-empirical relationship between longitudinal tire slip ratio ($\kappa$) and generated steady-state longitudinal tire force ($F_x$) under pure longitudinal motion (zero lateral slip angle, $\alpha = 0$).

This baseline formulation serves as the mechanical building block prior to introducing environmental road-condition parameterization (dry, wet, snow) and vertical load-dependent coefficient modeling in subsequent research phases.

---

## 2. Mathematical Formulation

The steady-state longitudinal tire force is modeled using the constant-coefficient Pacejka Magic Formula (sine-based empirical formulation):

$$F_x = F_z \cdot D \cdot \sin\left( C \cdot \arctan\left( B\kappa - E\left( B\kappa - \arctan(B\kappa) \right) \right) \right)$$

---

## 3. Variable and Parameter Definitions

### Variables
| Symbol | Description | SI Unit | Dimension |
| :--- | :--- | :--- | :--- |
| $F_x$ | Longitudinal tire force (tractive or braking) | $\text{N}$ | Force ($\text{M}\cdot\text{L}\cdot\text{T}^{-2}$) |
| $F_z$ | Vertical normal wheel load ($F_z > 0$) | $\text{N}$ | Force ($\text{M}\cdot\text{L}\cdot\text{T}^{-2}$) |
| $\kappa$ | Longitudinal slip ratio | $-$ | Dimensionless |

### Magic Formula Parameters
| Parameter | Name | Typical Physical Interpretation |
| :--- | :--- | :--- |
| $B$ | Stiffness Factor | Controls the initial curve slope in combination with $C$, $D$, and $F_z$. |
| $C$ | Shape Factor | Controls the general shape and stretch of the Magic Formula curve (typically $1.65 \le C \le 1.95$ for longitudinal force). |
| $D$ | Peak Factor | Scales the peak longitudinal force ($F_{x,\text{peak}} = D \cdot F_z$); in this normalized constant-coefficient formulation, $D$ corresponds to the peak friction coefficient $\mu_{x,\text{peak}}$. |
| $E$ | Curvature Factor | Controls the curvature around the peak force and the post-peak saturation/fall-off behavior ($E \le 1.0$). |

> **Interpretation of $D$ in Baseline vs. Full Formulation:** In the constant-coefficient Magic Formula adopted here, $D$ scales the peak longitudinal force directly and corresponds to the constant peak friction coefficient. This interpretation should not be generalized directly to the full load-dependent Magic Formula, where the peak factor/friction coefficient is itself a non-linear function of vertical load, camber, and inflation pressure.

---

## 4. Small-Slip Initial Stiffness

At small slip ratios ($\kappa \approx 0$), linearizing the trigonometric and arctangent functions ($\arctan(x) \approx x$ and $\sin(x) \approx x$) yields the initial longitudinal slip stiffness $K_0$:

$$\left. \frac{\partial F_x}{\partial \kappa} \right|_{\kappa = 0} = B \cdot C \cdot D \cdot F_z = K_0 \quad [\text{N}]$$

> **Important Note:** The stiffness factor $B$ alone does not represent the complete tire stiffness. The true initial longitudinal stiffness is the combined product of $B$, $C$, $D$, and normal load $F_z$.

---

## 5. Adopted Slip-Ratio Convention

The input slip ratio $\kappa$ follows the Phase 1A forward-motion convention:

$$\kappa = \frac{R\omega - V_x}{\max(|V_x|, \epsilon)}$$

Under this convention and positive parameter sets ($B, C, D > 0$):
* **$\kappa = 0$:** Pure rolling $\implies F_x = 0\,\text{N}$.
* **$\kappa > 0$:** Driving slip (wheel over-spinning) $\implies F_x > 0\,\text{N}$ (forward tractive force).
* **$\kappa < 0$:** Braking slip (wheel under-spinning) $\implies F_x < 0\,\text{N}$ (retarding braking force).

The function is strictly anti-symmetric (odd function): $F_x(-\kappa) = -F_x(\kappa)$.

---

## 6. Baseline Parameter Set for Model Verification

The following parameter set is utilized strictly for mathematical and qualitative verification of the baseline code implementation:

| Parameter | Value | Label / Classification |
| :--- | :--- | :--- |
| $F_z$ | $4000.0\,\text{N}$ | Representative passenger car wheel normal load |
| $B$ | $10.0$ | Representative stiffness parameter |
| $C$ | $1.9$ | Representative longitudinal shape factor |
| $D$ | $1.0$ | Representative peak factor ($\mu_{x,\text{peak}} = 1.0$) |
| $E$ | $0.97$ | Representative curvature factor |

> **Research Integrity Notice:** These numerical values represent an illustrative, representative parameter set for code verification. They are **not** experimental measurements of any specific commercial tire.

---

## 7. Model Assumptions

1. **Constant Parameters:** Coefficients $B, C, D, E$ are treated as independent of normal load, speed, inflation pressure, and temperature.
2. **Pure Longitudinal Slip:** Lateral slip angle is zero ($\alpha = 0$); camber angle is zero ($\gamma = 0$).
3. **Quasi-Steady Contact Patch:** Dynamic carcass deformation and relaxation lengths are neglected.
4. **Homogeneous Contact Surface:** Frictional characteristics are assumed uniform across the contact footprint.

---

## 8. Verification Methodology

The verification suite [matlab/simulation/verifyMagicFormulaLongitudinal.m](file:///c:/MATLAB/VehicleMobilityResearch/matlab/simulation/verifyMagicFormulaLongitudinal.m) evaluates 8 criteria:
1. **Zero Slip Check:** Confirms $F_x = 0$ at $\kappa = 0$.
2. **Driving Slip Sign:** Confirms $F_x > 0$ for $\kappa > 0$.
3. **Braking Slip Sign & Symmetry:** Confirms $F_x < 0$ and $F_x(-\kappa) = -F_x(\kappa)$.
4. **Vectorized Calculation:** Evaluates array inputs against an independent scalar loop.
5. **Normal Load Scaling:** Confirms linear scaling with $F_z$ for fixed $B,C,D,E$.
6. **Non-Positive Load Rejection:** Confirms rejection of $F_z \le 0$.
7. **Dimension Consistency:** Confirms rejection of mismatched non-scalar shapes.
8. **Small-Slip Slope:** Compares numerical finite-difference slope at $\Delta\kappa = 10^{-5}$ against the analytical value $K_0 = B \cdot C \cdot D \cdot F_z$.

---

## 9. Model Limitations

1. **No Load-Sensitivity:** In physical tires, the peak friction coefficient $\mu_{\text{peak}} = D$ and stiffness decrease non-linearly with increasing normal load (tire load sensitivity).
2. **No Combined Slip:** Tractive forces decrease when concurrent cornering forces are generated (friction ellipse / friction circle coupling).
3. **No Environmental Parameterization:** Surface-dependent friction scaling (such as wet pavement hydroplaning or snow shear deformation) is deferred to Phase 2.

---

## 10. References

1. **Pacejka, H. B.** (2012). *Tire and Vehicle Dynamics* (3rd ed.). Butterworth-Heinemann / Elsevier. (Chapter 4: "Semi-Empirical Tyre Models").
2. **Bakker, E., Nyborg, L., & Pacejka, H. B.** (1987). *Tyre Modelling for Use in Vehicle Dynamics Studies*. SAE Technical Paper 870421. https://doi.org/10.4271/870421.
3. **Rajamani, R.** (2012). *Vehicle Dynamics and Control* (2nd ed.). Springer US. (Chapter 3: "Lateral Vehicle Dynamics and Tire Models").
4. **The MathWorks, Inc.** (2026). *Longitudinal Wheel Forces (Magic Formula)*. Vehicle Dynamics Blockset Documentation.
