function [state_deriv, diagnostics] = closedLoopVehicleDerivative(state, delta, omega, params, pure_params, combined_params, solver_options)
% CLOSEDLOOPVEHICLEDERIVATIVE Evaluate the closed-loop 3-DOF planar vehicle state derivative.
%
%   [state_deriv, diagnostics] = closedLoopVehicleDerivative(state, delta, omega)
%   [state_deriv, diagnostics] = closedLoopVehicleDerivative(state, delta, omega, params)
%   [state_deriv, diagnostics] = closedLoopVehicleDerivative(state, delta, omega, params, pure_params, combined_params)
%   [state_deriv, diagnostics] = closedLoopVehicleDerivative(state, delta, omega, params, pure_params, combined_params, solver_options)
%
%   Computes the time derivative of the 3-DOF planar vehicle state:
%       state       = [vx; vy; r]   (3x1)
%       state_deriv = [dot_vx; dot_vy; dot_r]   (3x1)
%
%   The closed-loop coupling is resolved by solveQuasiStaticForceBalance(),
%   which iterates to find self-consistent (ax, ay) such that the tire forces
%   implied by the dynamic normal loads produce exactly those accelerations.
%
%   Physical acceleration definitions:
%       ax = Fx_total / m  =  dot_vx - vy * r
%       ay = Fy_total / m  =  dot_vy + vx * r
%
%   Inputs:
%       state           - 3x1 vehicle body state [vx; vy; r]
%                         vx : longitudinal body velocity [m/s]
%                         vy : lateral body velocity [m/s]
%                         r  : yaw rate [rad/s]
%       delta           - Steering angle [rad] (scalar or 4x1)
%       omega           - Wheel angular velocities [rad/s] (4x1)
%                         Wheel order: [FL; FR; RL; RR]
%       params          - (Optional) Rover parameter struct (roverParameters)
%       pure_params     - (Optional) Pure-slip tire parameter struct
%       combined_params - (Optional) Combined-slip tire parameter struct
%       solver_options  - (Optional) Struct for solveQuasiStaticForceBalance:
%                         .tol        - Convergence tolerance [m/s^2] (default: 1e-6)
%                         .max_iter   - Maximum iterations (default: 50)
%                         .a_init     - Initial acceleration guess [ax0; ay0]
%                         .relaxation - Under-relaxation factor in (0, 1]
%
%   Outputs:
%       state_deriv     - 3x1 time derivative [dot_vx; dot_vy; dot_r]
%       diagnostics     - Struct containing:
%                         .ax          - Converged longitudinal acceleration [m/s^2]
%                         .ay          - Converged lateral acceleration [m/s^2]
%                         .dot_r       - Yaw acceleration [rad/s^2]
%                         .Fx_wheel    - 4x1 longitudinal tire forces in wheel frame [N]
%                         .Fy_wheel    - 4x1 lateral tire forces in wheel frame [N]
%                         .Fz_wheel    - 4x1 dynamic normal loads [N]
%                         .kappa       - 4x1 longitudinal slip ratios [-]
%                         .alpha       - 4x1 slip angles [rad]
%                         .Gxa         - 4x1 combined-slip longitudinal weighting factors [-]
%                         .Gyk         - 4x1 combined-slip lateral weighting factors [-]
%                         .dynamics    - Struct from fourWheelDynamics (forces, moments)
%                         .solver_diag - Struct with convergence metadata
%
%   Wheel ordering convention (FL=1, FR=2, RL=3, RR=4):
%       Index 1: FL (Front-Left)
%       Index 2: FR (Front-Right)
%       Index 3: RL (Rear-Left)
%       Index 4: RR (Rear-Right)
%
%   Error identifiers:
%       solveQuasiStaticForceBalance:maxIterationsExceeded
%       dynamicNormalLoadDistribution:negativeNormalLoad

    % -----------------------------------------------------------------------
    % Default parameters
    % -----------------------------------------------------------------------
    if nargin < 4 || isempty(params)
        params = roverParameters();
    end
    if nargin < 5
        pure_params = [];
    end
    if nargin < 6
        combined_params = [];
    end
    if nargin < 7
        solver_options = [];
    end

    % -----------------------------------------------------------------------
    % Input extraction
    % -----------------------------------------------------------------------
    state = state(:);
    if numel(state) ~= 3
        error('closedLoopVehicleDerivative:invalidState', ...
              'state must be a 3-element vector [vx; vy; r].');
    end

    vx = state(1);
    vy = state(2);
    r  = state(3);

    % -----------------------------------------------------------------------
    % Phase 3F: resolve closed-loop algebraic force-balance
    % -----------------------------------------------------------------------
    [ax, ay, Fx_wheel, Fy_wheel, Fz_wheel, kappa, alpha, Gxa, Gyk, solver_diag] = ...
        solveQuasiStaticForceBalance(vx, vy, r, delta, omega, params, ...
                                     pure_params, combined_params, solver_options);

    % -----------------------------------------------------------------------
    % 3-DOF equations of motion (Phase 3A: fourWheelDynamics)
    % -----------------------------------------------------------------------
    [state_deriv, dynamics] = fourWheelDynamics(vx, vy, r, delta, Fx_wheel, Fy_wheel, params);

    % -----------------------------------------------------------------------
    % Assemble diagnostics
    % -----------------------------------------------------------------------
    diagnostics.ax          = ax;
    diagnostics.ay          = ay;
    diagnostics.dot_r       = state_deriv(3);
    diagnostics.Fx_wheel    = Fx_wheel;
    diagnostics.Fy_wheel    = Fy_wheel;
    diagnostics.Fz_wheel    = Fz_wheel;
    diagnostics.kappa       = kappa;
    diagnostics.alpha       = alpha;
    diagnostics.Gxa         = Gxa;
    diagnostics.Gyk         = Gyk;
    diagnostics.dynamics    = dynamics;
    diagnostics.solver_diag = solver_diag;
end
