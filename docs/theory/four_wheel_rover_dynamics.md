# Four-Wheel Rover Kinematics and 3-DOF Planar Vehicle Dynamics

## 1. Scope and Modeling Objectives

This document establishes the theoretical formulation for the planar kinematics and 3-DOF rigid-body dynamics of a four-wheel autonomous ground rover. 

The framework is structured around an independent four-wheel architecture:
- $\text{FL}$ — Front-Left (Wheel 1)
- $\text{FR}$ — Front-Right (Wheel 2)
- $\text{RL}$ — Rear-Left (Wheel 3)
- $\text{RR}$ — Rear-Right (Wheel 4)

Rather than collapsing the vehicle into a single-track (bicycle) approximation, each wheel maintains its own position vector, local kinematic velocity, steering angle, slip ratio, slip angle, and force transformation. This formulation establishes the kinematic and dynamic foundation required to study individual wheel interactions and spatial terrain variations.

---

## 2. Coordinate System and Sign Conventions

### 2.1 Vehicle Body-Fixed Frame
The rover body-fixed coordinate system has its origin at the vehicle Center of Gravity (CG):
- **$+x$ axis:** Points forward along the longitudinal vehicle centerline.
- **$+y$ axis:** Points leftward along the lateral vehicle axis.
- **$+z$ axis:** Points vertically upward, completing a right-handed orthogonal Cartesian triad.

The planar body velocity vector at the CG is denoted by:
$$\boldsymbol{v}_{\text{CG}} = \begin{bmatrix} v_x \\ v_y \\ 0 \end{bmatrix}$$

The vehicle angular velocity vector about the CG is:
$$\boldsymbol{\omega} = \begin{bmatrix} 0 \\ 0 \\ r \end{bmatrix}$$
where $r = \dot{\psi}$ is the yaw rate, defined positive for counter-clockwise (CCW) rotation when viewed from above ($+z$).

```
                +x (Forward)
                     ^
                     |
         FL [1]      |      FR [2]
        +-----+      |     +-----+
        |  |  |      |     |  |  |
        +-----+      |     +-----+
           \         |        /
            \        |       /
             \       |      /
              \      |     /
   +y (Left) <-------+ (CG)
              \      |
               \     |
                \    |
                 \   |
        +-----+      |     +-----+
        |  |  |      |     |  |  |
        +-----+      |     +-----+
         RL [3]      |      RR [4]
```

### 2.2 Relationship to Reference Standards
Vehicle dynamics literature commonly references SAE J670 and ISO 8855 terminology. SAE J670 historically uses a $+z$-down convention ($+x$ forward, $+y$ right, $+z$ down). To maintain standard consistency with modern robotics, planar navigation, and right-handed vector calculus, this project explicitly adopts the $+z$-up convention ($+x$ forward, $+y$ left, $+z$ up), while adopting standard SAE J670 terminology for vehicle states and kinematics.

---

## 3. Rover Geometry and Wheel Positions

The wheel positions relative to the vehicle CG in the body-fixed frame are defined as follows:

| Wheel | Designation | Index | Longitudinal Coordinate ($x_i$) | Lateral Coordinate ($y_i$) | Position Vector ($\boldsymbol{r}_i$) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| Front-Left | FL | 1 | $+l_f$ | $+t_f / 2$ | $[+l_f, +t_f/2, 0]^T$ |
| Front-Right | FR | 2 | $+l_f$ | $-t_f / 2$ | $[+l_f, -t_f/2, 0]^T$ |
| Rear-Left | RL | 3 | $-l_r$ | $+t_r / 2$ | $[-l_r, +t_r/2, 0]^T$ |
| Rear-Right | RR | 4 | $-l_r$ | $-t_r/ 2$ | $[-l_r, -t_r/2, 0]^T$ |

where:
- $l_f$: Distance from vehicle CG to the front axle centerline $[\text{m}]$.
- $l_r$: Distance from vehicle CG to the rear axle centerline $[\text{m}]$.
- $t_f$: Front axle track width $[\text{m}]$.
- $t_r$: Rear axle track width $[\text{m}]$.
- $L = l_f + l_r$: Total vehicle wheelbase $[\text{m}]$.

---

## 4. Rigid-Body Wheel Kinematics

The velocity of each wheel center $\boldsymbol{v}_{\text{body}, i}$ expressed in the vehicle body-fixed frame is derived from rigid-body planar kinematics:

$$\boldsymbol{v}_{\text{body}, i} = \boldsymbol{v}_{\text{CG}} + \boldsymbol{\omega} \times \boldsymbol{r}_i$$

Expanding the cross product:
$$\begin{bmatrix} v_{x,\text{body},i} \\ v_{y,\text{body},i} \\ 0 \end{bmatrix} = \begin{bmatrix} v_x \\ v_y \\ 0 \end{bmatrix} + \begin{bmatrix} 0 \\ 0 \\ r \end{bmatrix} \times \begin{bmatrix} x_i \\ y_i \\ 0 \end{bmatrix} = \begin{bmatrix} v_x - r y_i \\ v_y + r x_i \\ 0 \end{bmatrix}$$

Explicitly for each wheel:
$$\begin{aligned}
v_{x,\text{body},\text{FL}} &= v_x - r \left(\frac{t_f}{2}\right), \quad &v_{y,\text{body},\text{FL}} &= v_y + r l_f \\
v_{x,\text{body},\text{FR}} &= v_x + r \left(\frac{t_f}{2}\right), \quad &v_{y,\text{body},\text{FR}} &= v_y + r l_f \\
v_{x,\text{body},\text{RL}} &= v_x - r \left(\frac{t_r}{2}\right), \quad &v_{y,\text{body},\text{RL}} &= v_y - r l_r \\
v_{x,\text{body},\text{RR}} &= v_x + r \left(\frac{t_r}{2}\right), \quad &v_{y,\text{body},\text{RR}} &= v_y - r l_r
\end{aligned}$$

---

## 5. Front Steering and Wheel-Frame Velocity Transformation

### 5.1 Steering Input
Phase 3A adopts a front-steered rover configuration where front wheels share a common steer angle $\delta$:
$$\delta_{\text{FL}} = \delta, \quad \delta_{\text{FR}} = \delta, \quad \delta_{\text{RL}} = 0, \quad \delta_{\text{RR}} = 0$$
Ackermann geometric steering adjustments are excluded at this stage as a simplifying assumption and documented for future extension.

### 5.2 Wheel-Frame Transformation
The coordinate frame of wheel $i$ is rotated relative to the vehicle body frame by the steer angle $\delta_i$ about the $+z$ axis. The velocity vector in the wheel coordinate frame $[v_{x,\text{wheel},i}, v_{y,\text{wheel},i}]^T$ is obtained via the 2D rotation matrix:

$$\begin{bmatrix} v_{x,\text{wheel},i} \\ v_{y,\text{wheel},i} \end{bmatrix} = \begin{bmatrix} \cos\delta_i & \sin\delta_i \\ -\sin\delta_i & \cos\delta_i \end{bmatrix} \begin{bmatrix} v_{x,\text{body},i} \\ v_{y,\text{body},i} \end{bmatrix}$$

Expanding the components:
$$\begin{aligned}
v_{x,\text{wheel},i} &= \cos\delta_i \cdot v_{x,\text{body},i} + \sin\delta_i \cdot v_{y,\text{body},i} \\
v_{y,\text{wheel},i} &= -\sin\delta_i \cdot v_{x,\text{body},i} + \cos\delta_i \cdot v_{y,\text{body},i}
\end{aligned}$$

---

## 6. Slip Ratio and Wheel Slip Angle

### 6.1 Longitudinal Slip Ratio ($\kappa_i$)
The longitudinal slip ratio $\kappa_i$ for each individual wheel is calculated using the validated Phase 1A model ([longitudinalSlipRatio.m](file:///c:/MATLAB/VehicleMobilityResearch/matlab/tire/longitudinalSlipRatio.m)):

$$\kappa_i = \frac{R \omega_i - v_{x,\text{wheel},i}}{\max\left(|v_{x,\text{wheel},i}|, \epsilon_{\text{slip}}\right)}$$

where:
- $R$: Effective wheel rolling radius $[\text{m}]$.
- $\omega_i$: Rotational angular velocity of wheel $i$ $[\text{rad/s}]$.
- $\epsilon_{\text{slip}}$: Low-speed regularization threshold $[\text{m/s}]$ (default: $0.1\,\text{m/s}$).

### 6.2 Wheel Velocity Slip Angle ($\alpha_i$)
The signed wheel slip angle $\alpha_i$ is computed in the wheel coordinate frame by:

$$\alpha_i = \text{atan2}\left(v_{y,\text{wheel},i}, v_{x,\text{wheel},i}\right)$$

#### Project Definition and Sign Convention:
- Wheel frame: $+x$ forward along the wheel heading, $+y$ leftward perpendicular to the wheel center plane.
- $\alpha_i > 0$: Wheel velocity vector has a positive leftward lateral component ($v_{y,\text{wheel},i} > 0$).
- $\alpha_i < 0$: Wheel velocity vector has a negative rightward lateral component ($v_{y,\text{wheel},i} < 0$).
- Example: For a rover traveling forward ($v_{x,\text{body}} > 0, v_{y,\text{body}} = 0$) with front wheels steered to the left ($\delta > 0$), the relative ground velocity in the wheel frame is $v_{y,\text{wheel}} = -v_{x,\text{body}}\sin\delta < 0$, giving $\alpha < 0$.
- Low-Speed Regularization: For velocity magnitudes below $v_{\text{threshold}} = 10^{-4}\,\text{m/s}$ (including standstill $v_x = 0, v_y = 0$), $\alpha_i$ is set to $0.0\,\text{rad}$ to prevent numerical ill-conditioning and discontinuous angle values.

> **Convention Clarification:** This is the project's explicitly defined signed kinematic velocity angle convention. Literature sources define tire slip angles with varying signs (e.g., SAE J670, ISO 8855, Pacejka). The lateral tire-force constitutive relation and its sign convention will be formulated separately in Phase 3B.

---

## 7. Static Normal Load Distribution

Under static gravitational equilibrium, total vehicle weight $F_{z,\text{total}} = m g$ is distributed across axles based on longitudinal CG placement:

$$\begin{aligned}
F_{z,\text{front}} &= m g \left(\frac{l_r}{l_f + l_r}\right) \\
F_{z,\text{rear}}  &= m g \left(\frac{l_f}{l_f + l_r}\right)
\end{aligned}$$

For the Phase 3A baseline equilibrium case, equal left/right static normal-load distribution is assumed:
$$\begin{aligned}
F_{z,\text{FL}} &= F_{z,\text{FR}} = \frac{F_{z,\text{front}}}{2} = \frac{m g l_r}{2(l_f + l_r)} \\
F_{z,\text{RL}} &= F_{z,\text{RR}} = \frac{F_{z,\text{rear}}}{2}  = \frac{m g l_f}{2(l_f + l_r)}
\end{aligned}$$

Equilibrium condition:
$$\sum_{i=1}^4 F_{z,i} = F_{z,\text{FL}} + F_{z,\text{FR}} + F_{z,\text{RL}} + F_{z,\text{RR}} = m g$$

> **Important Architectural Distinction:**
> Equal left/right normal loads ($F_{z,\text{FL}} = F_{z,\text{FR}}$ and $F_{z,\text{RL}} = F_{z,\text{RR}}$) are assumed solely for static baseline equilibrium because the vehicle CG is laterally centered and dynamic lateral/longitudinal load transfer is not yet modeled.
>
> This does **not** collapse the vehicle into a symmetric or single-track (bicycle) model. All four wheels remain independently represented in the architecture:
> - Wheel velocities ($v_{x,\text{wheel},i}, v_{y,\text{wheel},i}$) remain independent.
> - Wheel slip ratios ($\kappa_i$) and slip angles ($\alpha_i$) remain independent.
> - Longitudinal forces ($F_{x,\text{wheel},i}$) and lateral forces ($F_{y,\text{wheel},i}$) remain independent.
> - Yaw moment contributions ($M_{z,i}$) are calculated independently for each wheel location.
>
> Preserving individual wheel states maintains the explicit four-wheel architecture required for future research into asymmetric normal loading, differential traction, and spatially non-uniform terrain conditions.

---

## 8. Force Transformation and Total Body Force

Tire tractive/braking forces $F_{x,\text{wheel},i}$ and lateral forces $F_{y,\text{wheel},i}$ generated at the contact patch are defined in the wheel coordinate frame. These forces are transformed into the vehicle body-fixed frame through rotation by steer angle $\delta_i$:

$$\begin{bmatrix} F_{x,\text{body},i} \\ F_{y,\text{body},i} \end{bmatrix} = \begin{bmatrix} \cos\delta_i & -\sin\delta_i \\ \sin\delta_i & \cos\delta_i \end{bmatrix} \begin{bmatrix} F_{x,\text{wheel},i} \\ F_{y,\text{wheel},i} \end{bmatrix}$$

Expanding the components:
$$\begin{aligned}
F_{x,\text{body},i} &= \cos\delta_i \cdot F_{x,\text{wheel},i} - \sin\delta_i \cdot F_{y,\text{wheel},i} \\
F_{y,\text{body},i} &= \sin\delta_i \cdot F_{x,\text{wheel},i} + \cos\delta_i \cdot F_{y,\text{wheel},i}
\end{aligned}$$

The total external forces acting on the vehicle body along the longitudinal and lateral axes are:
$$F_{x,\text{total}} = \sum_{i=1}^4 F_{x,\text{body},i}, \quad F_{y,\text{total}} = \sum_{i=1}^4 F_{y,\text{body},i}$$

---

## 9. Individual Wheel Yaw Moment and Total Moment

The yaw moment $M_{z,i}$ contributed by each individual wheel about the vehicle CG is determined by the planar cross product $\boldsymbol{M}_{z,i} = \boldsymbol{r}_i \times \boldsymbol{F}_{\text{body},i}$:

$$M_{z,i} = x_i \cdot F_{y,\text{body},i} - y_i \cdot F_{x,\text{body},i}$$

Explicitly for the four wheel positions:
$$\begin{aligned}
M_{z,\text{FL}} &= (+l_f) \cdot F_{y,\text{body},\text{FL}} - \left(+\frac{t_f}{2}\right) \cdot F_{x,\text{body},\text{FL}} \\
M_{z,\text{FR}} &= (+l_f) \cdot F_{y,\text{body},\text{FR}} - \left(-\frac{t_f}{2}\right) \cdot F_{x,\text{body},\text{FR}} \\
M_{z,\text{RL}} &= (-l_r) \cdot F_{y,\text{body},\text{RL}} - \left(+\frac{t_r}{2}\right) \cdot F_{x,\text{body},\text{RL}} \\
M_{z,\text{RR}} &= (-l_r) \cdot F_{y,\text{body},\text{RR}} - \left(-\frac{t_r}{2}\right) \cdot F_{x,\text{body},\text{RR}}
\end{aligned}$$

Total vehicle yaw moment:
$$M_{z,\text{total}} = \sum_{i=1}^4 M_{z,i}$$

This individual-wheel formulation preserves differential longitudinal forces (such as skid-steering or asymmetric tractive inputs) and differential lateral cornering moments.

---

## 10. 3-DOF Planar Equations of Motion

Applying Newton-Euler equations of motion in the rotating body-fixed reference frame ($\boldsymbol{\omega} = [0, 0, r]^T$):

The acceleration of the CG expressed in the body frame is:
$$\boldsymbol{a}_{\text{CG}} = \dot{\boldsymbol{v}}_{\text{CG}} + \boldsymbol{\omega} \times \boldsymbol{v}_{\text{CG}} = \begin{bmatrix} \dot{v}_x - v_y r \\ \dot{v}_y + v_x r \\ 0 \end{bmatrix}$$

The resulting 3-DOF planar equations of motion are:
$$\begin{aligned}
m \left(\dot{v}_x - v_y r\right) &= \sum_{i=1}^4 F_{x,\text{body},i} = F_{x,\text{total}} \\
m \left(\dot{v}_y + v_x r\right) &= \sum_{i=1}^4 F_{y,\text{body},i} = F_{y,\text{total}} \\
I_z \dot{r} &= \sum_{i=1}^4 M_{z,i} = M_{z,\text{total}}
\end{aligned}$$

Solving for the state time derivatives $[\dot{v}_x, \dot{v}_y, \dot{r}]^T$:
$$\begin{aligned}
\dot{v}_x &= \frac{F_{x,\text{total}}}{m} + v_y r \\
\dot{v}_y &= \frac{F_{y,\text{total}}}{m} - v_x r \\
\dot{r}   &= \frac{M_{z,\text{total}}}{I_z}
\end{aligned}$$

---

## 11. References

1. **SAE International.** (2022). *Vehicle Dynamics Terminology* (SAE Recommended Practice J670_202206). SAE International.
2. **Li, B., Raksincharoensak, H., & Wang, R.** (2022). Estimation of Longitudinal Force, Sideslip Angle and Yaw Rate for Four-Wheel Independent Actuated Autonomous Vehicles Based on PWA Tire Model. *Sensors*, 22(3), 885.
3. **Gillespie, T. D.** (1992). *Fundamentals of Vehicle Dynamics*. Society of Automotive Engineers, Warrendale, PA.
4. **Rajamani, R.** (2012). *Vehicle Dynamics and Control* (2nd ed.). Springer US.
