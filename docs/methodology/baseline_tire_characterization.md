# Baseline Longitudinal Tire Force Characterization

## 1. Objective

The objective of this methodology is to simulate, numerically characterize, and visualize the baseline longitudinal tire-force response ($F_x$) as a function of the longitudinal slip ratio ($\kappa$) across the full operational range $\kappa \in [-1.0, 1.0]$.

This characterization establishes the reference force-slip curve, initial stiffness, peak force location, and post-peak saturation behavior of the baseline tire model prior to introducing environmental road conditions (dry, wet, snow) and load sensitivity.

---

## 2. Models Used

The characterization relies strictly on the validated Phase 1 computational chain without modifying or duplicating underlying physics equations:
- [longitudinalSlipRatio.m](file:///c:/MATLAB/VehicleMobilityResearch/matlab/tire/longitudinalSlipRatio.m) — Kinematic longitudinal slip calculation with low-speed regularization.
- [magicFormulaLongitudinal.m](file:///c:/MATLAB/VehicleMobilityResearch/matlab/tire/magicFormulaLongitudinal.m) — Constant-coefficient Pacejka Magic Formula formulation:
  $$F_x = F_z \cdot D \cdot \sin\left( C \cdot \arctan\left( B\kappa - E\left( B\kappa - \arctan(B\kappa) \right) \right) \right)$$

---

## 3. Baseline Parameters

The characterization utilizes the established representative baseline parameter set:

| Parameter | Symbol | Value | Classification / Role |
| :--- | :--- | :--- | :--- |
| Vertical Normal Load | $F_z$ | $4000.0\,\text{N}$ | Representative wheel normal load |
| Stiffness Factor | $B$ | $10.0$ | Controls initial force-slip slope |
| Shape Factor | $C$ | $1.90$ | Controls curve profile and horizontal stretch |
| Peak Factor | $D$ | $1.00$ | Normalized peak friction coefficient ($\mu_{\text{peak}} = D$) |
| Curvature Factor | $E$ | $0.97$ | Controls curvature near and beyond peak |

> **Research Integrity Notice:** These numerical parameters are representative baseline values used for model evaluation. They do **not** represent experimental measurements of any specific commercial tire.

---

## 4. Simulation Setup and Numerical Resolution

- **Slip-Ratio Range:** $\kappa \in [-1.0, +1.0]$ covering pure rolling ($\kappa = 0$), locked-wheel braking ($\kappa = -1.0$), and 100% spin driving ($\kappa = +1.0$).
- **Discretization:** Uniform resolution with $N = 2001$ evaluation points ($\Delta\kappa = 1.0 \times 10^{-3}$), providing fine spatial resolution to resolve the peak region accurately.
- **Execution Script:** [matlab/simulation/characterizeBaselineTireForce.m](file:///c:/MATLAB/VehicleMobilityResearch/matlab/simulation/characterizeBaselineTireForce.m).

---

## 5. Numerical Characterization Results

The simulation evaluated the model and extracted the key performance metrics summarized below:

| Metric | Symbol / Formula | Computed Value | Units |
| :--- | :--- | :--- | :--- |
| Analytical Initial Tangent Stiffness | $K_0 = B \cdot C \cdot D \cdot F_z$ | $76000.0$ | $\text{N}/\text{unit slip}$ |
| Peak Driving Force | $F_{x,\text{max}}$ | $+4000.00$ | $\text{N}$ |
| Slip Ratio at Peak Driving Force | $\kappa_{\text{pos\_peak}}$ | $+0.1800$ | $-$ |
| Peak Braking Force Magnitude | $|F_{x,\text{min}}|$ | $4000.00$ | $\text{N}$ |
| Slip Ratio at Peak Braking Force | $\kappa_{\text{neg\_peak}}$ | $-0.1800$ | $-$ |
| Effective Peak Friction Coefficient | $\mu_{\text{peak}} = |F_{x,\text{peak}}| / F_z$ | $1.0000$ | $-$ |
| Terminal Braking Force ($\kappa = -1.0$) | $F_x(-1.0)$ | $-3658.09$ | $\text{N}$ |
| Terminal Driving Force ($\kappa = +1.0$) | $F_x(+1.0)$ | $+3658.09$ | $\text{N}$ |
| Sliding Force Ratio (Slide/Peak) | $F_x(1.0) / F_{x,\text{peak}}$ | $0.9145$ ($91.45\%$) | $-$ |

---

## 6. Physical Verification and Curve Interpretation

![Baseline Longitudinal Tire Force](file:///c:/MATLAB/VehicleMobilityResearch/results/figures/baseline_longitudinal_tire_force.png)

### Observations from the Characterization Curve:
1. **Linear Small-Slip Region ($|\kappa| \le 0.05$):**
   Near zero slip, the force-slip relationship is approximately linear, with a slope consistent with the analytical initial stiffness $K_0 = 76000\,\text{N}/\text{slip}$.
2. **Transition and Peak Region ($0.05 < |\kappa| \le 0.18$):**
   As slip increases, the model transitions from the approximately linear region toward the peak-force region, where the force-slip gradient decreases and the predicted longitudinal force approaches its maximum ($F_x = 4000.0\,\text{N}$) at $\kappa = \pm 0.18$.
3. **Post-Peak Saturation and Fall-off ($|\kappa| > 0.18$):**
   Beyond the peak slip ratio ($|\kappa| > 0.18$), the model exhibits post-peak force reduction. The predicted force gradually declines from $4000.0\,\text{N}$ to $3658.09\,\text{N}$ at $\kappa = \pm 1.0$ (an $8.55\%$ reduction from peak to terminal slip), governed by the empirical curvature factor $E = 0.97$.
4. **Odd Symmetry:**
   Because no horizontal or vertical shifts ($S_h = 0, S_v = 0$) are present, the response exhibits exact odd symmetry ($F_x(-\kappa) = -F_x(\kappa)$), producing identical peak force magnitudes for tractive acceleration and braking.

---

## 7. Artifacts Generated

1. **Figure:** [results/figures/baseline_longitudinal_tire_force.png](file:///c:/MATLAB/VehicleMobilityResearch/results/figures/baseline_longitudinal_tire_force.png)
2. **Data File:** [results/data/baseline_longitudinal_tire_force.csv](file:///c:/MATLAB/VehicleMobilityResearch/results/data/baseline_longitudinal_tire_force.csv) ($2001$ rows containing discretized $\kappa$ and corresponding $F_x$).

---

## 8. Limitations

1. **Simulation-Only Model:** These are numerical simulation results from a simplified, constant-coefficient Magic Formula. They do not constitute experimental measurements or validation against a physical test tire.
2. **No Load Dependency:** $B, C, D, E$ are constant; real-world tire friction coefficients decrease non-linearly with increasing normal load ($F_z$).
3. **No Environmental Coupling:** Frictional reduction due to wet surfaces (hydroplaning) or snow compaction/shearing is not included in this baseline baseline model.
