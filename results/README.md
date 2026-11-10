# Tire Grip and Four-Wheel Vehicle Dynamics on Snow

### A MATLAB-based preliminary computational study of tire forces, vehicle response, and simplified snow-chain sensitivity

## Overview

This project began with a practical research question:

> **How does additional traction provided by snow chains affect tire forces and the resulting behavior of a four-wheel vehicle on snow?**

I independently developed a MATLAB-based vehicle-dynamics framework incrementally, beginning with tire-force fundamentals and progressing toward a simplified four-wheel model, time-domain simulation, trajectory analysis, and a controlled SnowChain benchmark.

This is a **preliminary mathematical study**. It demonstrates understanding of tire-force modelling, vehicle dynamics, simulation, verification, and research limitations. It does not claim experimentally validated snow-chain performance.

## Research Workflow

```text
Tire-force model
       ↓
Four-wheel tire kinematics
       ↓
Longitudinal and lateral tire forces
       ↓
Combined-slip formulation
       ↓
Dynamic normal-load transfer
       ↓
Closed-loop vehicle dynamics
       ↓
Time-domain simulation
       ↓
Vehicle trajectory integration
       ↓
Performance and handling analysis
       ↓
Snow vs. simplified SnowChain comparison
```

## What Was Implemented

- Longitudinal slip-ratio calculation
- Magic Formula longitudinal tire-force modelling
- Four-wheel vehicle kinematics
- Lateral tire-force modelling
- Combined longitudinal/lateral slip
- Dynamic normal-load transfer
- Closed-loop planar vehicle dynamics
- Time-domain numerical simulation
- Global vehicle trajectory integration
- Vehicle performance and handling metrics
- Modular snow-condition parameter framework
- Simplified SnowChain sensitivity benchmark
- Component-level verification and regression testing

The model uses a planar vehicle representation with prescribed wheel-speed and steering inputs. It does not attempt to reproduce a production vehicle, tire-testing rig, or high-fidelity snow-contact solver.

## Phase 3K: Simplified SnowChain Benchmark

The benchmark compares:

1. **Snow** — the baseline parameterized snow condition
2. **SnowChain** — a controlled sensitivity case with increased longitudinal Magic Formula peak-factor scaling

Only the longitudinal pure-slip parameter was modified. The lateral and combined-slip parameter sets were retained to isolate the intended modelling change.

The selected snow pure-slip reference set is **B = 5.0, C = 2.0, D = 0.30, E = 1.0**. The same values are provisionally used for lateral pure-slip behaviour as a symmetry assumption, not as a separately validated lateral snow-tire set. SnowChain applies the existing illustrative 1.5× multiplier to longitudinal `Dx`, giving `Dx = 0.45`. This is an **illustrative modelling assumption**, not an experimentally measured coefficient set for a particular tire, chain, snow depth, or temperature.

## Selected Results

| Metric | Snow | SnowChain | Difference |
|---|---:|---:|---:|
| Acceleration final speed | 3.868 m/s | 3.912 m/s | +0.044 m/s (+1.1%) |
| Acceleration peak longitudinal force | 58.021 N | 58.681 N | +0.660 N (+1.1%) |
| Braking distance | 4.664 m | 4.609 m | −0.055 m (−1.2%) |
| Cornering peak lateral force | 67.682 N | 67.712 N | +0.030 N (+0.044%) |
| Cornering peak yaw rate | 0.159 rad/s | 0.135 rad/s | −0.025 rad/s (−15.5%) |
| Final lateral deviation | 1.142 m | 1.007 m | −0.135 m (−11.8%) |


### Straight braking

![Straight braking distance comparison](results/reference_snow_run/figures/3K_braking_distance.png)

| Case | Braking distance |
|---|---:|
| Snow | 4.664 m |
| SnowChain benchmark | 4.609 m |

This corresponds to an approximate **1.2% reduction** under the selected manoeuvre and model assumptions.

This is a model-sensitive comparison, not a prediction of real-world snow-chain braking performance.

### Constant-cornering trajectory

![Constant cornering trajectory comparison](results/reference_snow_run/figures/3K_cornering_trajectory.png)

The two cases produced different simulated lateral trajectories. This demonstrates that changing longitudinal tire-force behaviour can influence coupled vehicle response through the vehicle-dynamics equations.

The result is described conservatively as an **altered yaw response and lateral trajectory**, not proof of improved stability or superior real-world handling.

### Dynamic longitudinal force versus slip

![Dynamic longitudinal force versus slip](results/reference_snow_run/figures/3K_fx_vs_kappa.png)

This figure shows the dynamic longitudinal-force response of the front-left wheel during the benchmark manoeuvre.

### Dynamic force trace

![Dynamic force trace](results/reference_snow_run/figures/3K_force_utilization.png)

This figure shows the evolving longitudinal and lateral tire-force components for the front-left wheel during cornering.

It is a force-utilization diagnostic—not a measured friction boundary, friction ellipse, or experimentally established tire-capacity envelope.

## Main Numerical Observations

Under the selected simulation conditions:

- Straight-acceleration final speed increased from approximately **3.868 m/s** to **3.912 m/s**
- Peak longitudinal force increased from approximately **58.021 N** to **58.681 N**
- Straight-braking distance decreased from approximately **4.664 m** to **4.609 m**
- Peak lateral force during constant cornering changed from **67.682 N** to **67.712 N**
- Peak yaw rate changed from **0.159 rad/s** to **0.135 rad/s**; final lateral deviation changed from **1.142 m** to **1.007 m**
- The differences arose from the implemented tire-force sensitivity and its coupling to the four-wheel vehicle equations

These observations are specific to the current model, manoeuvres, parameter values, and numerical setup.

## Model Scope and Limitations

The current framework does not explicitly model:

- Snow compaction, shear, or digging
- Tread-block deformation
- Snow depth, temperature, water content, or microstructure
- Tire rubber-compound temperature dependence
- Detailed chain geometry or chain–snow contact
- Wheel inertia and drivetrain torque dynamics
- Pitch, roll, or heave dynamics
- Experimental calibration or validation

Snow is represented through parameterized tire-force behaviour. The SnowChain case is a controlled sensitivity modification rather than a complete physical model of a chained tire.

## Verification

The project was developed incrementally with component-level verification suites covering tire modelling, vehicle kinematics, force generation, load transfer, vehicle dynamics, simulation, trajectory integration, and analysis.

The completed regression record reported:

```text
Revised Phase 3K: 4 / 4 tests passed
Closed-Loop Vehicle Dynamics (3F): 22 / 22 tests passed
Full regression suite: not rerun after the coefficient revision
```

The repository retains the component verification scripts. The consolidated result was obtained by running them sequentially; a single unified regression runner is not currently included.

## Reproducibility

Principal MATLAB entry points:

```text
matlab/analysis/runSnowChainBenchmark.m
matlab/analysis/runSnowChainResults.m
matlab/simulation/verifySnowChainBenchmark.m
```

Figures are stored under:

```text
results/reference_snow_run/figures/
```

The earlier illustrative plots are retained under `results/original_illustrative_run/`.

Detailed numerical results should be documented in:

```text
results/README.md
```

Supporting theory documents are stored under:

```text
docs/theory/
```

## Future Research Direction

The next stage would investigate:

- Tire–snow interaction
- Tire–ice interaction, including very low-friction black-ice conditions
- The influence of tread characteristics on traction
- Experimental tire and vehicle testing
- Tire-force parameter identification
- Calibration against measured data
- Validation of simplified vehicle models
- More physically grounded snow and ice contact models

A major learning objective is understanding how researchers connect simplified computational models with experimental observations and progressively improve model fidelity.

## References

The project is informed by literature on empirical tire-force modelling, combined tire forces, vehicle dynamics, tire–snow interaction, tire–ice interaction, snow-chain traction, and experimental parameter identification.

A detailed reference list should be maintained in:

```text
docs/references.md
```

## Project Status

**Status:** Preliminary computational research study completed through the simplified SnowChain benchmark.

The implementation is intentionally limited in scope, establishing a foundation in tire mechanics, four-wheel vehicle dynamics, numerical simulation, and research methodology before progressing toward advanced snow and ice interaction modelling.
