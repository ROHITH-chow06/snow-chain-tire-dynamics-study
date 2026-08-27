function [ax, ay, Fx_wheel, Fy_wheel, Fz_wheel, kappa, alpha, Gxa, Gyk, diagnostics] = solveQuasiStaticForceBalance(vx, vy, r, delta, omega, params, pure_params, combined_params, solver_options)
% SOLVEQUASISTATICFORCEBALANCE Resolves the coupled acceleration/normal-load algebraic loop.
%
%   [ax, ay, Fx_wheel, Fy_wheel, Fz_wheel, kappa, alpha, Gxa, Gyk, diagnostics] = solveQuasiStaticForceBalance(vx, vy, r, delta, omega)
%   [...] = solveQuasiStaticForceBalance(vx, vy, r, delta, omega, params)
%   [...] = solveQuasiStaticForceBalance(vx, vy, r, delta, omega, params, pure_params, combined_params)
%   [...] = solveQuasiStaticForceBalance(vx, vy, r, delta, omega, params, pure_params, combined_params, solver_options)
%
%   Solves the quasi-static fixed-point problem:
%       (ax, ay) <-> Fz(ax, ay) <-> (Fx, Fy) <-> (ax, ay)
%   where physical accelerations are defined as:
%       ax = sum(Fx_body) / m
%       ay = sum(Fy_body) / m
%
%   Inputs:
%       vx              - Longitudinal vehicle CG body velocity [m/s]
%       vy              - Lateral vehicle CG body velocity [m/s]
%       r               - Vehicle yaw rate [rad/s]
%       delta           - Steering angle [rad] (scalar or 4x1)
%       omega           - Wheel rotational speeds [rad/s] (4x1 or 1x4)
%       params          - (Optional) Rover parameter struct from roverParameters()
%       pure_params     - (Optional) Pure-slip tire parameter struct
%       combined_params - (Optional) Combined-slip tire parameter struct
%       solver_options  - (Optional) Struct with fields:
%                         .tol        - Convergence tolerance [m/s^2] (default: 1e-6)
%                         .max_iter   - Maximum iterations (default: 50)
%                         .a_init     - Initial acceleration guess [ax0; ay0] (default: [0; 0])
%                         .relaxation - Under-relaxation factor in (0, 1] (default: 1.0)
%
%   Outputs:
%       ax, ay          - Converged physical longitudinal and lateral accelerations [m/s^2]
%       Fx_wheel        - 4x1 vector of converged wheel-frame longitudinal tire forces [N]
%       Fy_wheel        - 4x1 vector of converged wheel-frame lateral tire forces [N]
%       Fz_wheel        - 4x1 vector of converged dynamic normal loads [N]
%       kappa, alpha    - 4x1 vectors of longitudinal slip ratios and slip angles
%       Gxa, Gyk        - 4x1 vectors of combined-slip weighting factors
%       diagnostics     - Struct with solver convergence metadata

    % Default parameters
    if nargin < 6 || isempty(params)
        params = roverParameters();
    end
    if nargin < 7 || isempty(pure_params)
        pure_params = struct('Bx', 10.0, 'Cx', 1.90, 'Dx', 1.00, 'Ex', 0.97, ...
                             'By', 10.0, 'Cy', 1.30, 'Dy', 1.00, 'Ey', -1.00);
    end
    if nargin < 8 || isempty(combined_params)
        combined_params = struct('Bx_alpha', 10.0, 'Cx_alpha', 1.00, 'Ex_alpha', -1.00, 'SHx_alpha', 0.0, ...
                                 'By_kappa', 10.0, 'Cy_kappa', 1.00, 'Ey_kappa', -1.00, 'SHy_kappa', 0.0);
    end

    % Default solver options
    tol        = 1e-6;
    max_iter   = 50;
    a_init     = [0.0; 0.0];
    relaxation = 1.0;

    if nargin >= 9 && ~isempty(solver_options)
        if isfield(solver_options, 'tol'),        tol        = solver_options.tol; end
        if isfield(solver_options, 'max_iter'),   max_iter   = solver_options.max_iter; end
        if isfield(solver_options, 'a_init'),     a_init     = solver_options.a_init; end
        if isfield(solver_options, 'relaxation'), relaxation = solver_options.relaxation; end
    end

    % 1. Evaluate four-wheel contact kinematics (constant for fixed vehicle state & controls)
    kinematics = wheelKinematics(vx, vy, r, delta, omega, params);
    kappa = kinematics.kappa;
    alpha = kinematics.alpha;

    % 2. Fixed-point iteration to resolve dynamic normal load and acceleration coupling
    ax = a_init(1);
    ay = a_init(2);
    converged = false;
    history = zeros(max_iter, 3); % [iter, ax_eval, ay_eval]

    for k = 1:max_iter
        % Evaluate dynamic normal loads at current acceleration estimate
        Fz_wheel = dynamicNormalLoadDistribution(params, ax, ay);

        % Evaluate combined-slip tire forces
        [Fx_wheel, Fy_wheel, Gxa, Gyk] = combinedSlipTireForce(kappa, alpha, Fz_wheel, pure_params, combined_params);

        % Transform forces into vehicle body frame
        cos_d = cos(kinematics.delta);
        sin_d = sin(kinematics.delta);
        Fx_body = cos_d .* Fx_wheel - sin_d .* Fy_wheel;
        Fy_body = sin_d .* Fx_wheel + cos_d .* Fy_wheel;

        % Evaluate resulting physical accelerations
        ax_eval = sum(Fx_body) / params.m;
        ay_eval = sum(Fy_body) / params.m;

        history(k, :) = [k, ax_eval, ay_eval];
        err = max(abs([ax_eval - ax; ay_eval - ay]));

        if err < tol
            converged = true;
            ax = ax_eval;
            ay = ay_eval;
            iter_count = k;
            break;
        end

        % Update iterate with optional under-relaxation
        ax = (1.0 - relaxation) * ax + relaxation * ax_eval;
        ay = (1.0 - relaxation) * ay + relaxation * ay_eval;
        iter_count = k;
    end

    if ~converged
        error('solveQuasiStaticForceBalance:maxIterationsExceeded', ...
              'Fixed-point solver did not converge within %d iterations (last error = %.3e m/s^2).', max_iter, err);
    end

    % Final evaluation at converged acceleration to ensure exact consistency
    Fz_wheel = dynamicNormalLoadDistribution(params, ax, ay);
    [Fx_wheel, Fy_wheel, Gxa, Gyk] = combinedSlipTireForce(kappa, alpha, Fz_wheel, pure_params, combined_params);

    cos_d = cos(kinematics.delta);
    sin_d = sin(kinematics.delta);
    Fx_body = cos_d .* Fx_wheel - sin_d .* Fy_wheel;
    Fy_body = sin_d .* Fx_wheel + cos_d .* Fy_wheel;
    ax = sum(Fx_body) / params.m;
    ay = sum(Fy_body) / params.m;

    diagnostics.converged   = converged;
    diagnostics.final_error = err;
    diagnostics.iterations  = iter_count;
    diagnostics.history     = history(1:iter_count, :);
end
