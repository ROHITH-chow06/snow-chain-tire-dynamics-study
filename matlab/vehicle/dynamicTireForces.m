function [Fx_wheel, Fy_wheel, Fz_wheel, kappa, alpha, Gxa, Gyk, kinematics] = dynamicTireForces(vx, vy, r, delta, omega, ax, ay, params, pure_params, combined_params, epsilon_slip, v_threshold)
% DYNAMICTIREFORCES Integrated four-wheel dynamic normal-load and combined-slip tire force evaluation.
%
%   [Fx_wheel, Fy_wheel, Fz_wheel, kappa, alpha, Gxa, Gyk, kinematics] = dynamicTireForces(vx, vy, r, delta, omega, ax, ay)
%   [...] = dynamicTireForces(vx, vy, r, delta, omega, ax, ay, params)
%   [...] = dynamicTireForces(vx, vy, r, delta, omega, ax, ay, params, pure_params, combined_params)
%   [...] = dynamicTireForces(vx, vy, r, delta, omega, ax, ay, params, pure_params, combined_params, epsilon_slip, v_threshold)
%
%   Evaluates kinematics, dynamic normal load distribution, and combined-slip
%   Magic Formula tire forces for all four wheels of a planar ground rover:
%       Index 1: FL (Front-Left)
%       Index 2: FR (Front-Right)
%       Index 3: RL (Rear-Left)
%       Index 4: RR (Rear-Right)
%
%   Inputs:
%       vx              - Vehicle CG longitudinal body velocity [m/s] (scalar)
%       vy              - Vehicle CG lateral body velocity [m/s] (scalar)
%       r               - Vehicle yaw rate [rad/s] (scalar, positive CCW)
%       delta           - Front steering angle [rad] (scalar, or 4x1 vector)
%       omega           - Wheel rotational speeds [rad/s] (4x1 or 1x4 vector)
%       ax              - Current vehicle longitudinal acceleration [m/s^2] (scalar)
%       ay              - Current vehicle lateral acceleration [m/s^2] (scalar)
%       params          - (Optional) Rover parameter struct from roverParameters()
%       pure_params     - (Optional) Pure-slip tire parameters struct (Bx, Cx, Dx, Ex, By, Cy, Dy, Ey)
%       combined_params - (Optional) Combined-slip tire parameters struct (Bx_alpha, Cx_alpha, Ex_alpha, SHx_alpha, ...)
%       epsilon_slip    - (Optional) Slip ratio regularization threshold [m/s] (default: 0.1)
%       v_threshold     - (Optional) Slip angle zero-speed threshold [m/s] (default: 1e-4)
%
%   Outputs:
%       Fx_wheel        - 4x1 vector of longitudinal tire forces in wheel frame [N]
%       Fy_wheel        - 4x1 vector of lateral tire forces in wheel frame [N]
%       Fz_wheel        - 4x1 vector of dynamic normal loads [N]
%       kappa           - 4x1 vector of longitudinal slip ratios [-]
%       alpha           - 4x1 vector of signed wheel slip angles [rad]
%       Gxa             - 4x1 vector of longitudinal weighting factors [-]
%       Gyk             - 4x1 vector of lateral weighting factors [-]
%       kinematics      - Struct containing 4-wheel kinematic fields from wheelKinematics()
%
%   Coupling Boundary:
%       This function implements explicit quasi-static dynamic normal load
%       evaluation using the supplied accelerations (ax, ay). Fully coupled
%       implicit force-acceleration iteration is deferred to future work.

    % Default parameters
    if nargin < 8 || isempty(params)
        params = roverParameters();
    end
    if nargin < 9 || isempty(pure_params)
        pure_params = struct('Bx', 10.0, 'Cx', 1.90, 'Dx', 1.00, 'Ex', 0.97, ...
                             'By', 10.0, 'Cy', 1.30, 'Dy', 1.00, 'Ey', -1.00);
    end
    if nargin < 10 || isempty(combined_params)
        combined_params = struct('Bx_alpha', 10.0, 'Cx_alpha', 1.00, 'Ex_alpha', -1.00, 'SHx_alpha', 0.0, ...
                                 'By_kappa', 10.0, 'Cy_kappa', 1.00, 'Ey_kappa', -1.00, 'SHy_kappa', 0.0);
    end
    if nargin < 11 || isempty(epsilon_slip)
        epsilon_slip = 0.1;
    end
    if nargin < 12 || isempty(v_threshold)
        v_threshold = 1e-4;
    end

    % Validate acceleration inputs
    validateattributes(ax, {'numeric'}, {'real', 'scalar'}, mfilename, 'ax', 6);
    validateattributes(ay, {'numeric'}, {'real', 'scalar'}, mfilename, 'ay', 7);

    % 1. Determine four-wheel kinematics (Phase 3A)
    kinematics = wheelKinematics(vx, vy, r, delta, omega, params, epsilon_slip, v_threshold);
    kappa = kinematics.kappa;
    alpha = kinematics.alpha;

    % 2. Determine quasi-static dynamic normal loads (Phase 3D)
    Fz_wheel = dynamicNormalLoadDistribution(params, ax, ay);

    % 3. Evaluate combined-slip Magic Formula tire forces (Phase 3C)
    [Fx_wheel, Fy_wheel, Gxa, Gyk] = combinedSlipTireForce(kappa, alpha, Fz_wheel, pure_params, combined_params);
end
