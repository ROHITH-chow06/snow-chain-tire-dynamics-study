# Road Condition Comparative Longitudinal Tire Modeling

## 1. Research Question

How do adverse surface environmental conditions (specifically transition from dry tarmac to wet pavement and snow-covered roads) influence the fundamental longitudinal force generation, peak available friction, initial slip stiffness, and post-peak saturation behavior of a pneumatic tire?

---

## 2. Experimental Setup and Controlled Variables

To isolate the effect of surface conditions, all other vehicle and tire parameters are held strictly constant:
- **Vertical Tire Normal Load:** $F_z = 4000.0\,\text{N}$
- **Kinematic Slip Range:** $\kappa \in [-1.0, +1.0]$
- **Discretization Resolution:** $N = 2001$ points ($\Delta\kappa = 0.001$)
- **Underlying Tire Model:** Constant-coefficient Pacejka Magic Formula ([magicFormulaLongitudinal.m](file:///c:/MATLAB/VehicleMobilityResearch/matlab/tire/magicFormulaLongitudinal.m))
- **Execution Script:** [compareRoadConditions.m](file:///c:/MATLAB/VehicleMobilityResearch/matlab/simulation/compareRoadConditions.m)

---

## 3. Road-Condition Parameter Sets

The parameters are sourced from established literature and technical documentation (MathWorks Vehicle Dynamics Blockset):

| Surface Condition | Stiffness Factor ($B$) | Shape Factor ($C$) | Peak Factor ($D$) | Curvature Factor ($E$) | Peak Friction ($\mu_{\text{peak}} = D$) | Initial Stiffness ($K_0 = BCDF_z$) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Dry Tarmac** | $10.0$ | $1.90$ | $1.00$ | $0.97$ | $1.00$ | $76\,000\,\text{N}/\text{slip}$ |
| **Wet Tarmac** | $12.0$ | $2.30$ | $0.82$ | $1.00$ | $0.82$ | $90\,528\,\text{N}/\text{slip}$ |
| **Snow** | $5.0$ | $2.00$ | $0.30$ | $1.00$ | $0.30$ | $12\,000\,\text{N}/\text{slip}$ |

> **Research Integrity Notice:** These parameter sets represent representative baseline engineering models. They are **not** experimental measurements of any specific physical tire tested in this project.

---

## 4. Quantitative Results and Comparison

![Comparative Longitudinal Tire Force](file:///c:/MATLAB/VehicleMobilityResearch/results/figures/road_condition_longitudinal_tire_force.png)

### Summary of Comparative Metrics ($F_z = 4000\,\text{N}$)

| Metric | Dry Tarmac | Wet Tarmac | Snow Road | Wet vs. Dry ($\Delta\%$) | Snow vs. Dry ($\Delta\%$) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Peak Longitudinal Force ($F_{x,\text{peak}}$)** | $4000.00\,\text{N}$ | $3280.00\,\text{N}$ | $1200.00\,\text{N}$ | **$-18.00\%$** | **$-70.00\%$** |
| **Slip at Peak Force ($\kappa_{\text{peak}}$)** | $+0.1800$ | $+0.0880$ | $+0.3110$ | $-51.11\%$ | $+72.78\%$ |
| **Effective Peak Friction ($\mu_{\text{peak}}$)** | $1.0000$ | $0.8200$ | $0.3000$ | **$-18.00\%$** | **$-70.00\%$** |
| **Initial Stiffness ($K_0$)** | $76\,000\,\text{N}/\text{slip}$ | $90\,528\,\text{N}/\text{slip}$ | $12\,000\,\text{N}/\text{slip}$ | **$+19.12\%$** | **$-84.21\%$** |
| **Force at $\kappa = 0.05$** | $2942.48\,\text{N}$ | $2979.71\,\text{N}$ | $554.66\,\text{N}$ | $+1.27\%$ | **$-81.15\%$** |
| **Force at $\kappa = 0.10$** | $3823.37\,\text{N}$ | $3268.47\,\text{N}$ | $915.87\,\text{N}$ | $-14.51\%$ | **$-76.05\%$** |
| **Force at $\kappa = 0.20$** | $3996.71\,\text{N}$ | $2993.26\,\text{N}$ | $1165.82\,\text{N}$ | $-25.11\%$ | **$-70.83\%$** |

![Peak Longitudinal Force Comparison](file:///c:/MATLAB/VehicleMobilityResearch/results/figures/road_condition_peak_force_comparison.png)

---

## 5. Model-Based Results and Curve Interpretation

> **Methodological Clarification:**
> The comparative changes in peak force, peak slip location, and initial tangent stiffness are **model-based simulation results** resulting from the empirical Magic Formula parameterization. They represent the macroscopic mathematical response of the parameterized curves rather than direct experimental measurements.

1. **Dry Tarmac (Reference Baseline):**
   Exhibits a balanced baseline force-slip profile characterized by a peak longitudinal force ($F_{x,\text{peak}} = 4000.0\,\text{N}$) at $\kappa = 0.1800$, an initial stiffness $K_0 = 76\,000.0\,\text{N}/\text{slip}$, and a post-peak terminal force of $F_x(1.0) = 3658.09\,\text{N}$ ($91.45\%$ of peak).
2. **Wet Tarmac (Reduced Peak & Earlier Saturation):**
   - Predicted peak force decreases by $18.00\%$ ($3280.0\,\text{N}$, $\mu_{\text{peak}} = 0.82$) compared to the dry baseline.
   - The higher initial tangent stiffness ($K_0 = 90\,528.0\,\text{N}/\text{slip}$, $+19.12\%$) and earlier peak slip ($\kappa = 0.0880$) are direct mathematical consequences of the representative parameter combination ($B = 12.0, C = 2.30, D = 0.82 \implies K_0 = BCDF_z$). This should **not** be interpreted as experimental evidence that physical wet tires inherently possess higher structural stiffness than dry tires.
   - The model predicts a narrower initial rise range and earlier force saturation under this parameter set.
3. **Snow Road (Substantial Force Attenuation & Extended Compliance):**
   - Predicted peak tractive force is reduced by $70.00\%$ ($1200.0\,\text{N}$, $\mu_{\text{peak}} = 0.30$) relative to the dry baseline.
   - The lower stiffness factor ($B = 5.0, C = 2.0 \implies K_0 = 12\,000.0\,\text{N}/\text{slip}$, $-84.21\%$) shifts the peak force location outward to a higher slip ratio ($\kappa = 0.3110$, $+72.78\%$).
   - Across the low-slip operating range ($\kappa \le 0.10$), the model generates substantially lower available longitudinal force, reflecting the lower traction capacity parameterized for snow.

---

## 6. Generated Data Artifacts

1. **Full Discretized Dataset:** [results/data/road_condition_longitudinal_tire_force.csv](file:///c:/MATLAB/VehicleMobilityResearch/results/data/road_condition_longitudinal_tire_force.csv) ($2001$ rows with columns `kappa`, `Fx_dry_N`, `Fx_wet_N`, `Fx_snow_N`).
2. **Summary Metrics Table:** [results/data/road_condition_summary.csv](file:///c:/MATLAB/VehicleMobilityResearch/results/data/road_condition_summary.csv) (Parameterized comparison metrics and percentage changes).
3. **Figures:**
   - [results/figures/road_condition_longitudinal_tire_force.png](file:///c:/MATLAB/VehicleMobilityResearch/results/figures/road_condition_longitudinal_tire_force.png)
   - [results/figures/road_condition_peak_force_comparison.png](file:///c:/MATLAB/VehicleMobilityResearch/results/figures/road_condition_peak_force_comparison.png)

---

## 7. Model Assumptions and Limitations

1. **Simulation-Only Model:** These results reflect mathematical evaluations of constant-coefficient Magic Formula models with representative parameter sets, not experimental test-bench data.
2. **Speed-Independent Hydroplaning:** Real-world wet road friction decreases nonlinearly as vehicle forward velocity increases due to dynamic hydroplaning; in this constant-parameter model, $D$ is speed-invariant.
3. **Homogeneous Snow Surface:** Variations in snow moisture content, ambient temperature, fresh vs. hard-packed snow density, and tread pattern snow-clearing effects are not modeled at this stage.

---

## 8. Future Extension: Black Ice and High-Altitude Environments (Ladakh Roadmap)

Black-ice conditions are being considered as a future extension of the same tire-road interaction framework, with potential application to cold/high-altitude road environments such as Ladakh.

The present study does not introduce Ladakh-specific measurements or calibrated black-ice parameters. Inclusion of this condition will depend on the availability of suitable literature-supported or experimentally derived parameter data and the remaining project timeline.
