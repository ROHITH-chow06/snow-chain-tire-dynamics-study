function [time, state, diagnostics] = simulateVehicleTrajectory(tSpan, initialState, deltaInput, omegaInput, params, pure_params, combined_params, sim_options)
% SIMULATEVEHICLETRAJECTORY Numerical time integration of 6-state planar vehicle trajectory.
%
%   [time, state] = simulateVehicleTrajectory(tSpan, initialState, deltaInput, omegaInput)
%   [time, state] = simulateVehicleTrajectory(tSpan, initialState, deltaInput, omegaInput, params)
%   [time, state] = simulateVehicleTrajectory(tSpan, initialState, deltaInput, omegaInput, params, pure_params, combined_params)
%   [time, state, diagnostics] = simulateVehicleTrajectory(tSpan, initialState, deltaInput, omegaInput, params, pure_params, combined_params, sim_options)
%
%   Integrates the 6-state planar vehicle equations of motion over time
%   using MATLAB's adaptive Runge-Kutta (4,5) solver (ode45):
%       x = [X; Y; psi; vx; vy; r]
%
%   State definition and ordering:
%       State 1: X   - Inertial / global longitudinal position [m]
%       State 2: Y   - Inertial / global lateral position [m]
%       State 3: psi - Vehicle yaw heading angle [rad] (continuous, unwrapped)
%       State 4: vx  - Vehicle body-fixed longitudinal velocity [m/s]
%       State 5: vy  - Vehicle body-fixed lateral velocity [m/s]
%       State 6: r   - Vehicle yaw rate [rad/s]
%
%   Governing Equations:
%       Global pose kinematics:
%           dot(X)   = vx * cos(psi) - vy * sin(psi)
%           dot(Y)   = vx * sin(psi) + vy * cos(psi)
%           dot(psi) = r
%       Body dynamics (Phase 3F closedLoopVehicleDerivative):
%           [dot(vx); dot(vy); dot(r)] = closedLoopVehicleDerivative([vx; vy; r], delta(t), omega(t), ...)
%
%   Inputs:
%       tSpan           - Time span for integration [t0, tf] or specific time points [t0, t1, ..., tf] (1xN or Nx1)
%       initialState    - 6-element initial state vector [X0; Y0; psi0; vx0; vy0; r0]
%       deltaInput      - Front steering angle [rad]: scalar, 4x1 vector, or function handle @(t)
%       omegaInput      - Wheel angular speeds [rad/s]: 4x1/1x4 vector, or function handle @(t)
%                         Wheel order: [FL; FR; RL; RR]
%       params          - (Optional) Rover parameter struct from roverParameters()
%       pure_params     - (Optional) Pure-slip tire parameter struct
%       combined_params - (Optional) Combined-slip tire parameter struct
%       sim_options     - (Optional) Simulation configuration struct with fields:
%                         .RelTol         - Relative ODE tolerance (default: 1e-6)
%                         .AbsTol         - Absolute ODE tolerance (default: 1e-8)
%                         .solver_options - Solver options struct passed to solveQuasiStaticForceBalance
%
%   Numerical Solver:
%       ode45 (Dormand-Prince explicit Runge-Kutta (4,5) adaptive-step pair).
%       Default RelTol = 1e-6, AbsTol = 1e-8. Appropriate for smooth
%       continuous planar vehicle maneuvers.
%
%   Outputs:
%       time            - Nx1 vector of evaluated simulation time points [s]
%       state           - Nx6 matrix of vehicle state trajectory: [X, Y, psi, vx, vy, r]
%       diagnostics     - (Optional) Struct containing time histories of derived quantities:
%                         .ax          - Nx1 longitudinal accelerations [m/s^2]
%                         .ay          - Nx1 lateral accelerations [m/s^2]
%                         .dot_r       - Nx1 yaw accelerations [rad/s^2]
%                         .Fx_wheel    - Nx4 wheel-frame longitudinal tire forces [N]
%                         .Fy_wheel    - Nx4 wheel-frame lateral tire forces [N]
%                         .Fz_wheel    - Nx4 dynamic normal loads [N]
%                         .kappa       - Nx4 longitudinal slip ratios [-]
%                         .alpha       - Nx4 slip angles [rad]
%                         .Gxa         - Nx4 longitudinal weighting factors [-]
%                         .Gyk         - Nx4 lateral weighting factors [-]
%                         .Fx_total    - Nx1 total body longitudinal forces [N]
%                         .Fy_total    - Nx1 total body lateral forces [N]
%                         .Mz_total    - Nx1 total body yaw moments [N*m]
%                         .delta       - Nx4 or Nx1 steering angles [rad]
%                         .omega       - Nx4 wheel angular speeds [rad/s]
%
%   Wheel ordering:
%       Column 1: FL (Front-Left)
%       Column 2: FR (Front-Right)
%       Column 3: RL (Rear-Left)
%       Column 4: RR (Rear-Right)

    % -----------------------------------------------------------------------
    % Default parameters and options
    % -----------------------------------------------------------------------
    if nargin < 5 || isempty(params)
        params = roverParameters();
    end
    if nargin < 6
        pure_params = [];
    end
    if nargin < 7
        combined_params = [];
    end
    if nargin < 8
        sim_options = struct();
    end

    % -----------------------------------------------------------------------
    % Validate tSpan
    % -----------------------------------------------------------------------
    validateattributes(tSpan, {'numeric'}, {'real', 'vector'}, mfilename, 'tSpan', 1);
    if numel(tSpan) < 2
        error('simulateVehicleTrajectory:invalidTSpan', 'tSpan must contain at least 2 time points.');
    end
    if any(diff(tSpan) <= 0)
        error('simulateVehicleTrajectory:invalidTSpan', 'tSpan must be strictly monotonically increasing.');
    end

    % -----------------------------------------------------------------------
    % Validate initial state
    % -----------------------------------------------------------------------
    validateattributes(initialState, {'numeric'}, {'real', 'vector'}, mfilename, 'initialState', 2);
    if numel(initialState) ~= 6
        error('simulateVehicleTrajectory:invalidInitialState', ...
              'initialState must be a 6-element vector [X; Y; psi; vx; vy; r].');
    end
    x0 = initialState(:);

    % -----------------------------------------------------------------------
    % Standardize input handles delta(t) and omega(t)
    % -----------------------------------------------------------------------
    deltaFcn = wrapInputFunction(deltaInput, 'deltaInput', 1);
    omegaFcn = wrapInputFunction(omegaInput, 'omegaInput', 4);

    % -----------------------------------------------------------------------
    % Configure ODE options
    % -----------------------------------------------------------------------
    relTol = 1e-6;
    absTol = 1e-8;
    solver_opts = [];

    if isfield(sim_options, 'RelTol'),         relTol      = sim_options.RelTol; end
    if isfield(sim_options, 'RelativeTolerance'), relTol   = sim_options.RelativeTolerance; end
    if isfield(sim_options, 'AbsTol'),         absTol      = sim_options.AbsTol; end
    if isfield(sim_options, 'AbsoluteTolerance'), absTol   = sim_options.AbsoluteTolerance; end
    if isfield(sim_options, 'solver_options'), solver_opts = sim_options.solver_options; end

    odeOptions = odeset('RelTol', relTol, 'AbsTol', absTol);

    % -----------------------------------------------------------------------
    % Define the 6-state ODE derivative function
    % -----------------------------------------------------------------------
    odeFun = @(t, x) vehicleODE(t, x, deltaFcn, omegaFcn, params, pure_params, combined_params, solver_opts);

    % -----------------------------------------------------------------------
    % Execute numerical ODE integration (ode45 — adaptive Runge-Kutta (4,5))
    % -----------------------------------------------------------------------
    [time, state] = ode45(odeFun, tSpan, x0, odeOptions);

    % -----------------------------------------------------------------------
    % Post-process diagnostics if requested
    % -----------------------------------------------------------------------
    if nargout > 2
        N = length(time);
        diagnostics.ax       = zeros(N, 1);
        diagnostics.ay       = zeros(N, 1);
        diagnostics.dot_r    = zeros(N, 1);
        diagnostics.Fx_wheel = zeros(N, 4);
        diagnostics.Fy_wheel = zeros(N, 4);
        diagnostics.Fz_wheel = zeros(N, 4);
        diagnostics.kappa    = zeros(N, 4);
        diagnostics.alpha    = zeros(N, 4);
        diagnostics.Gxa      = zeros(N, 4);
        diagnostics.Gyk      = zeros(N, 4);
        diagnostics.Fx_total = zeros(N, 1);
        diagnostics.Fy_total = zeros(N, 1);
        diagnostics.Mz_total = zeros(N, 1);
        diagnostics.delta    = zeros(N, 4);
        diagnostics.omega    = zeros(N, 4);

        for k = 1:N
            tk = time(k);
            xk = state(k, :)';
            dk = deltaFcn(tk);
            if isscalar(dk)
                dk_vec = [dk; dk; 0.0; 0.0];
            else
                dk_vec = dk(:);
            end
            om_k = omegaFcn(tk);

            diagnostics.delta(k, :) = dk_vec';
            diagnostics.omega(k, :) = om_k(:)';

            [~, diag_k] = closedLoopVehicleDerivative(xk(4:6), dk, om_k, params, pure_params, combined_params, solver_opts);

            diagnostics.ax(k, 1)       = diag_k.ax;
            diagnostics.ay(k, 1)       = diag_k.ay;
            diagnostics.dot_r(k, 1)    = diag_k.dot_r;
            diagnostics.Fx_wheel(k, :) = diag_k.Fx_wheel';
            diagnostics.Fy_wheel(k, :) = diag_k.Fy_wheel';
            diagnostics.Fz_wheel(k, :) = diag_k.Fz_wheel';
            diagnostics.kappa(k, :)    = diag_k.kappa';
            diagnostics.alpha(k, :)    = diag_k.alpha';
            diagnostics.Gxa(k, :)      = diag_k.Gxa';
            diagnostics.Gyk(k, :)      = diag_k.Gyk';
            diagnostics.Fx_total(k, 1) = diag_k.dynamics.Fx_total;
            diagnostics.Fy_total(k, 1) = diag_k.dynamics.Fy_total;
            diagnostics.Mz_total(k, 1) = diag_k.dynamics.Mz_total;
        end
    end
end

% ===========================================================================
% Local function: 6-state ODE derivative
% ===========================================================================
function xdot = vehicleODE(t, x, deltaFcn, omegaFcn, params, pure_params, combined_params, solver_opts)
    % Extract 6-state components
    % x = [X; Y; psi; vx; vy; r]
    psi = x(3);
    vx  = x(4);
    vy  = x(5);
    r   = x(6);

    % Evaluate time-dependent inputs
    delta = deltaFcn(t);
    omega = omegaFcn(t);

    % 1. Global position and heading kinematics
    dot_X   = vx * cos(psi) - vy * sin(psi);
    dot_Y   = vx * sin(psi) + vy * cos(psi);
    dot_psi = r;

    % 2. Phase 3F Closed-loop planar vehicle dynamic derivatives
    sdot_3dof = closedLoopVehicleDerivative([vx; vy; r], delta, omega, params, pure_params, combined_params, solver_opts);

    % Assemble 6x1 state time derivative
    xdot = [
        dot_X;
        dot_Y;
        dot_psi;
        sdot_3dof(1);
        sdot_3dof(2);
        sdot_3dof(3)
    ];
end

% ===========================================================================
% Local helper: wrap numeric inputs or validate function handles
% ===========================================================================
function fcnHandle = wrapInputFunction(inputVal, inputName, expectedElements)
% Accepts either a function handle @(t) or a constant real numeric value.
% Numeric constants are wrapped into a constant anonymous function @(~) val.
% expectedElements: 1 for deltaInput (scalar or 4x1 accepted), 4 for omegaInput.
    if isa(inputVal, 'function_handle')
        % Verify function handle can be evaluated at t=0
        try
            testVal = inputVal(0.0);
        catch ME
            error('simulateVehicleTrajectory:invalidFunctionHandle', ...
                  'Function handle %s failed to evaluate at t=0: %s', inputName, ME.message);
        end
        if ~isnumeric(testVal) || ~isreal(testVal)
            error('simulateVehicleTrajectory:invalidInput', ...
                  'Function handle %s must return real numeric values.', inputName);
        end
        if expectedElements == 4 && ~(numel(testVal) == 4 || numel(testVal) == 1)
            error('simulateVehicleTrajectory:invalidInput', ...
                  '%s returned %d elements; expected 4 (or scalar).', inputName, numel(testVal));
        end
        fcnHandle = inputVal;
    elseif isnumeric(inputVal) && isreal(inputVal)
        val = inputVal;
        if expectedElements == 4 && ~(numel(val) == 4 || numel(val) == 1)
            error('simulateVehicleTrajectory:invalidInput', ...
                  '%s numeric input must be a 4-element vector (or scalar).', inputName);
        end
        if expectedElements == 1 && ~ismember(numel(val), [1, 4])
            error('simulateVehicleTrajectory:invalidInput', ...
                  '%s numeric input must be a scalar or 4-element vector.', inputName);
        end
        % Return an anonymous function that always returns the provided value
        fcnHandle = @(~) val;
    else
        error('simulateVehicleTrajectory:invalidInputType', ...
              '%s must be a real numeric array or a function handle @(t).', inputName);
    end
end
