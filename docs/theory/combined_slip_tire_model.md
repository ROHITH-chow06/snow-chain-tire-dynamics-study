# Phase 3C: Reduced Combined-Slip Magic Formula Tire Model

## 1. Physical Motivation for Combined Slip
A tire cannot generate its maximum pure longitudinal force and its maximum pure lateral force simultaneously. Because the total shear stress at the contact patch is bounded by available friction (often idealized as a friction circle or friction ellipse), the generation of lateral force intrinsically reduces the available longitudinal force, and vice versa. This phenomenon is known as combined slip. Incorporating combined slip is necessary to accurately model a vehicle under simultaneous cornering and braking/accelerating maneuvers.

## 2. Pure-Slip Constituent Models
This phase builds on the Phase 1B and Phase 3B pure-slip constituent models:
- **Pure Longitudinal Force ($F_{x0}$):** $F_{x0}(\kappa, F_z)$
- **Pure Lateral Force ($F_{y0}$):** $F_{y0}(\alpha, F_z)$

## 3. Reduced Combined-Slip Formulation
To model the force reduction, this implementation uses a **reduced combined-slip weighting formulation inspired by PAC2002 methodology**.
> [!IMPORTANT]
> This implementation is a reduced combined-slip weighting formulation inspired by PAC2002 methodology. It is not a complete PAC2002 tire model.

The combined forces are computed by multiplying the pure-slip forces by Pacejka-style weighting functions ($G_{x\alpha}$ and $G_{y\kappa}$):
$$F_x = G_{x\alpha}(\alpha) \cdot F_{x0}(\kappa, F_z)$$
$$F_y = G_{y\kappa}(\kappa) \cdot F_{y0}(\alpha, F_z)$$

## 4. Exact Weighting Equation
The PAC2002-style weighting function $G(x)$ uses a normalized cosine structure:
$$G(x) = \frac{\cos\left( C \cdot \arctan\left( B x_s - E \left( B x_s - \arctan(B x_s) \right) \right) \right)}{G_0}$$
where the shifted slip is:
$$x_s = x + S_H$$

## 5. Normalization Denominator
To ensure that $G(0) = 1.0$ precisely, the normalization denominator $G_0$ is evaluated at $x = 0$ (i.e. zero opposing slip):
$$G_0 = \cos\left( C \cdot \arctan\left( B S_H - E \left( B S_H - \arctan(B S_H) \right) \right) \right)$$

## 6. Parameter Meanings
- **$B$ (Stiffness Factor):** Dictates how rapidly the primary force decreases as the secondary (opposing) slip increases.
- **$C$ (Shape Factor):** Bounds the cosine argument.
- **$E$ (Curvature Factor):** Influences the curvature of the transition.
- **$S_H$ (Horizontal Shift):** Shifts the peak of the weighting function, capturing physical asymmetry in the combined slip curve.

## 7. Representative Parameter Provenance
> [!WARNING]
> The weighting coefficients used in this phase are representative values selected for mathematical characterization and verification and are not identified from experimental tire data. 

For the baseline verification, the following parameters are used for both $G_{x\alpha}$ and $G_{y\kappa}$:
- $B = 10.0$
- $C = 1.0$ (ensures argument stays within $[-\pi/2, \pi/2]$ bounds, ensuring monotonicity without clipping)
- $E = -1.0$
- $S_H = 0.0$ (symmetric model)

## 8. Boundary Conditions
- **Zero opposing slip:** $G(0) = 1.0$, which precisely recovers pure-slip forces.
- **Standstill:** $\kappa = 0, \alpha = 0 \implies F_x = 0, F_y = 0$.

## 9. Baseline Symmetry Properties
For the baseline $S_H = 0$, the function $G(x)$ is even, $G(-x) = G(x)$.
This propagates to the combined forces:
- **$F_x$ odd symmetry in $\kappa$:** $F_x(-\kappa, \alpha) = -F_x(\kappa, \alpha)$
- **$F_y$ odd symmetry in $\alpha$:** $F_y(\kappa, -\alpha) = -F_y(\kappa, \alpha)$
- **$F_x$ even symmetry in $\alpha$:** $F_x(\kappa, -\alpha) = F_x(\kappa, \alpha)$
- **$F_y$ even symmetry in $\kappa$:** $F_y(-\kappa, \alpha) = F_y(\kappa, \alpha)$

## 10. Non-zero $S_H$ Behavior
When $S_H \neq 0$, the weighting-function maximum occurs at the shifted location $x = -S_H$ for the present parameterization. Its normalized value is $1/G_0$, whose magnitude relative to 1 depends on the selected parameters. The normalization correctly handles this so that $G(0)$ remains exactly $1.0$, naturally avoiding artificial clipping (`min(G, 1)` or `max(G, 0)`) while maintaining the PAC2002 formulation integrity. The function is no longer generally even about $x = 0$.

## 11. Verification Results
The MATLAB automated test suite `verifyCombinedSlipTireForce.m` confirms:
- Precise recovery of pure longitudinal and lateral forces at zero opposing slip.
- Mathematical reduction behavior in all four quadrants of combined slip.
- Preservation of symmetry properties for the baseline $S_H = 0$ case.
- Numerical robustness (no NaN/Inf) over dense grids.

## 12. Limitations
- Constant representative coefficients are used (not empirical data).
- The formulation is a mathematically reduced model and not the full PAC2002 definition, which includes load dependencies, camber effects, turn slip, and additional coupled weighting interaction terms.
- Static normal load is still assumed.
