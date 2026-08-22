# Longitudinal Slip Ratio Model

## 1. Concept and Physical Meaning

When a pneumatic tire transmits tractive or braking forces, elastic deformation occurs within the tread rubber and carcass in the contact patch (tire footprint). As a consequence of this deformation and localized micro-slip, a kinematic velocity difference develops between the wheel circumferential speed ($R\omega$) and the vehicle longitudinal forward speed ($V_x$).

The **longitudinal slip ratio** ($\kappa$) is a dimensionless quantity that parameterizes this relative velocity difference. It serves as the fundamental kinematic input for semi-empirical and physical tire force models (such as Pacejka's Magic Formula) to determine the generated longitudinal tire force ($F_x$).

---

## 2. Adopted Mathematical Formulation

In this project, the longitudinal slip ratio is defined as:

$$\kappa = \frac{R\omega - V_x}{\max(|V_x|, \epsilon)}$$

where $\epsilon > 0$ is a small regularization parameter introduced to maintain numerical stability as vehicle forward velocity approaches zero ($V_x \to 0$).

---

## 3. Variable Definitions and Units

| Variable | Description | SI Unit | Dimension |
| :--- | :--- | :--- | :--- |
| $\kappa$ | Longitudinal slip ratio | $-$ | Dimensionless |
| $R$ | Effective tire rolling radius | $\text{m}$ | Length ($\text{L}$) |
| $\omega$ | Wheel angular velocity | $\text{rad/s}$ | Angular frequency ($\text{T}^{-1}$) |
| $V_x$ | Vehicle longitudinal velocity at wheel center | $\text{m/s}$ | Velocity ($\text{L}\cdot\text{T}^{-1}$) |
| $\epsilon$ | Low-speed regularization threshold | $\text{m/s}$ | Velocity ($\text{L}\cdot\text{T}^{-1}$) |

---

## 4. Sign Convention

Across vehicle dynamics literature, varying sign conventions exist for longitudinal slip (e.g., SAE J670, ISO 8855, and Pacejka formulations). This project adopts the following consistent definition:

* **$\kappa = 0$ (Pure Rolling):**
  The wheel circumferential speed equals the vehicle forward speed ($R\omega = V_x$). In an un-deflected, frictionless idealization, no longitudinal net force is generated.
* **$\kappa > 0$ (Driving / Acceleration Slip):**
  The wheel circumferential speed exceeds the vehicle speed ($R\omega > V_x$). The contact patch generates forward tractive force ($F_x > 0$) propelling the vehicle.
* **$\kappa < 0$ (Braking / Deceleration Slip):**
  The wheel circumferential speed is lower than the vehicle speed ($R\omega < V_x$). The contact patch generates retarding braking force ($F_x < 0$).
* **$\kappa = -1$ (Wheel Lockup):**
  When the wheel is completely locked under braking ($\omega = 0$) while the vehicle is moving at $V_x > \epsilon$, $\kappa = \frac{0 - V_x}{V_x} = -1.0$ (100% braking slip).

### Scope: Forward Vehicle Motion
The kinematic slip formulation and sign convention in Phase 1A are defined specifically for forward vehicle travel ($V_x > 0$). Reverse vehicle motion ($V_x < 0$), where velocity sign reversals require modified kinematics and directional references, is outside the current Phase 1A scope.

---

## 5. Low-Speed Numerical Regularization

At normal driving speeds ($|V_x| \ge \epsilon$), the equation reduces to the standard kinematic slip definition:

$$\kappa = \frac{R\omega - V_x}{|V_x|}$$

When the vehicle decelerates toward a standstill ($V_x \to 0$), the unregularized kinematic denominator approaches zero, leading to division-by-zero singularities ($\text{NaN}$ or $\text{Inf}$).

### Regularization Strategy
To ensure continuous numerical evaluation without division-by-zero singularities:
- A floor threshold $\epsilon$ (default: $0.1\text{ m/s}$) is applied to the denominator: $\text{denom} = \max(|V_x|, \epsilon)$.
- The regularization function $\max(|V_x|, \epsilon)$ is continuous ($C^0$) across all $V_x$, but exhibits a derivative slope discontinuity (non-smooth corner) at $|V_x| = \epsilon$.
- At complete standstill ($V_x = 0, \omega = 0$), the regularized equation produces $\kappa = 0$, reflecting zero steady-state slip.
- During low-speed transitions, $\epsilon$ bounds the denominator from below, avoiding numerical overflow while preserving the continuous sign of the slip.

---

## 6. Model Assumptions and Limitations

1. **Rigid / Effective Radius Assumption:** The rolling radius $R$ is treated as a constant effective radius. In reality, $R$ varies dynamically with normal load, tire inflation pressure, centrifugal expansion at high rotational speeds, and carcass compliance.
2. **Kinematic vs. Dynamic Slip:** The formula assumes quasi-steady-state kinematic slip. In high-frequency transients (such as rapid ABS cycling or abrupt traction spikes), tire carcass compliance and relaxation length dynamics ($L_r$) must be considered.
3. **Standstill Physics & Derivative Discontinuity:** While $\epsilon$-regularization prevents numerical failure at $V_x \approx 0$, it introduces a derivative slope change at $|V_x| = \epsilon$ and does not capture static friction (stiction) or elastic carcass deflection at rest.
4. **Unidirectional Formulation:** Kinematics are defined for forward vehicle travel ($V_x > 0$).

---

## 7. References

1. **Pacejka, H. B.** (2012). *Tire and Vehicle Dynamics* (3rd ed.). Butterworth-Heinemann / Elsevier. (Chapter 1: "Tire Characteristics and Vehicle Handling and Stability").
2. **Gillespie, T. D.** (1992). *Fundamentals of Vehicle Dynamics*. Society of Automotive Engineers (SAE), Warrendale, PA. (Chapter 10: "Tires").
3. **Rajamani, R.** (2012). *Vehicle Dynamics and Control* (2nd ed.). Springer US. (Chapter 3: "Lateral Vehicle Dynamics and Tire Models").
4. **SAE International.** (2008). *Vehicle Dynamics Terminology* (SAE J670 / ISO 8855). SAE Recommended Practice.
