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

The SnowChain modification is an **illustrative modelling assumption**, not an experimentally measured coefficient set for a particular tire, chain, snow depth, or temperature.

## Selected Results

### Straight braking

![Straight braking distance comparison](results/figures/3K_braking_distance.png)

| Case | Braking distance |
|---|---:|
| Snow | 4.850 m |
| SnowChain benchmark | 4.724 m |

This corresponds to an approximate **2.6% reduction** under the selected manoeuvre and model assumptions.

This is a model-sensitive comparison, not a prediction of real-world snow-chain braking performance.

### Constant-cornering trajectory

![Constant cornering trajectory comparison](results/figures/3K_cornering_trajectory.png)

The two cases produced different simulated lateral trajectories. This demonstrates that changing longitudinal tire-force behaviour can influence coupled vehicle response through the vehicle-dynamics equations.

The result is described conservatively as an **altered yaw response and lateral trajectory**, not proof of improved stability or superior real-world handling.

### Dynamic longitudinal force versus slip

![Dynamic longitudinal force versus slip](results/figures/3K_fx_vs_kappa.png)

This figure shows the dynamic longitudinal-force response of the front-left wheel during the benchmark manoeuvre.

### Dynamic force trace

![Dynamic force trace](results/figures/3K_force_utilization.png)

This figure shows the evolving longitudinal and lateral tire-force components for the front-left wheel during cornering.

It is a force-utilization diagnostic—not a measured friction boundary, friction ellipse, or experimentally established tire-capacity envelope.

## Main Numerical Observations

Under the selected simulation conditions:

- Straight-acceleration final speed increased from approximately **3.736 m/s** to **3.822 m/s**
- Peak longitudinal force increased from approximately **56.038 N** to **57.335 N**
- Straight-braking distance decreased from approximately **4.850 m** to **4.724 m**
- Peak lateral force during constant cornering remained nearly unchanged
- Peak yaw rate and final lateral deviation changed between the two cases
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
225 / 225 tests passed
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
results/figures/
```

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
