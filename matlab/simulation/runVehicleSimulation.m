function [stateHistory, time] = runVehicleSimulation(x0, steeringFcn, wheelSpeedFcn, tFinal, dt, params, pureParams, combinedParams)
% RUNVEHICLESIMULATION Fixed-step RK4 numerical integration of 3-DOF planar vehicle dynamics.
%
%   [stateHistory, time] = runVehicleSimulation(x0, steeringFcn, wheelSpeedFcn, tFinal, dt)
%   [stateHistory, time] = runVehicleSimulation(x0, steeringFcn, wheelSpeedFcn, tFinal, dt, params)
%   [stateHistory, time] = runVehicleSimulation(x0, steeringFcn, wheelSpeedFcn, tFinal, dt, params, pureParams, combinedParams)
%
%   Numerically integrates the 6-state planar vehicle equations of motion using
%   a classic 4th-order Runge-Kutta (RK4) fixed-step scheme:
%
%   State Vector:
%       x = [X; Y; psi; vx; vy; r]
%       Index 1: X   - Global inertial longitudinal position [m]
%       Index 2: Y   - Global inertial lateral position [m]
%       Index 3: psi - Vehicle heading angle [rad] (unwrapped, continuous)
%       Index 4: vx  - Vehicle body-fixed longitudinal velocity [m/s]
%       Index 5: vy  - Vehicle body-fixed lateral velocity [m/s]
%       Index 6: r   - Vehicle body-fixed yaw rate [rad/s]
%
%   Equations of Motion:
%       Global pose kinematics:
%           dot(X)   = vx * cos(psi) - vy * sin(psi)
%           dot(Y)   = vx * sin(psi) + vy * cos(psi)
%           dot(psi) = r
%       Body dynamics (evaluated directly via closedLoopVehicleDerivative):
%           [dot(vx); dot(vy); dot(r)] = closedLoopVehicleDerivative([vx; vy; r], delta(t), omega(t), params, ...)
%
%   Inputs:
%       x0             - 6-element initial state vector [X0; Y0; psi0; vx0; vy0; r0]
%       steeringFcn    - Steering angle function delta(t) [rad] or scalar/vector constant
%       wheelSpeedFcn  - Wheel angular velocity function omega(t) [rad/s] (4x1) or vector constant
%                        Wheel ordering: [FL; FR; RL; RR]
%       tFinal         - Final simulation time [s] (scalar > 0)
%       dt             - Fixed integration time step [s] (scalar > 0)
%       params         - (Optional) Rover parameter struct from roverParameters()
%       pureParams     - (Optional) Pure-slip tire parameter struct
%       combinedParams - (Optional) Combined-slip tire parameter struct
%
%   Outputs:
%       stateHistory   - N x 6 matrix of simulated state history across time
%       time           - N x 1 column vector of simulation time points [0, dt, 2*dt, ..., tFinal]'

    % -----------------------------------------------------------------------
    % Default parameters and options
    % -----------------------------------------------------------------------
    if nargin < 6 || isempty(params)
        params = roverParameters();
    end
    if nargin < 7
        pureParams = [];
    end
    if nargin < 8
        combinedParams = [];
    end

    % -----------------------------------------------------------------------
    % Input Validation
    % -----------------------------------------------------------------------
    validateattributes(x0, {'numeric'}, {'real', 'vector'}, mfilename, 'x0', 1);
    if numel(x0) ~= 6
        error('runVehicleSimulation:invalidInitialState', ...
              'x0 must be a 6-element vector [X; Y; psi; vx; vy; r].');
    end
    x_curr = x0(:);

    validateattributes(tFinal, {'numeric'}, {'real', 'scalar', 'positive'}, mfilename, 'tFinal', 4);
    validateattributes(dt, {'numeric'}, {'real', 'scalar', 'positive'}, mfilename, 'dt', 5);

    if dt > tFinal
        error('runVehicleSimulation:invalidTimeStep', 'Time step dt cannot exceed tFinal.');
    end

    % -----------------------------------------------------------------------
    % Standardize input functions
    % -----------------------------------------------------------------------
    stFcn = wrapInput(steeringFcn, 'steeringFcn', 1);
    wsFcn = wrapInput(wheelSpeedFcn, 'wheelSpeedFcn', 4);

    % -----------------------------------------------------------------------
    % Build time grid
    % -----------------------------------------------------------------------
    time = (0 : dt : tFinal)';
    if time(end) < tFinal && (tFinal - time(end)) > 1e-12
        time = [time; tFinal];
    end
    N = length(time);

    % -----------------------------------------------------------------------
    % Preallocate state history
    % -----------------------------------------------------------------------
    stateHistory = zeros(N, 6);
    stateHistory(1, :) = x_curr';

    % -----------------------------------------------------------------------
    % Fixed-step 4th-Order Runge-Kutta (RK4) Integration Loop
    % -----------------------------------------------------------------------
    for i = 1 : (N - 1)
        t_n = time(i);
        h   = time(i + 1) - t_n;

        % Stage 1
        k1 = vehicleDerivative(t_n, x_curr, stFcn, wsFcn, params, pureParams, combinedParams);

        % Stage 2
        x_k2 = x_curr + 0.5 * h * k1;
        k2 = vehicleDerivative(t_n + 0.5 * h, x_k2, stFcn, wsFcn, params, pureParams, combinedParams);

        % Stage 3
        x_k3 = x_curr + 0.5 * h * k2;
        k3 = vehicleDerivative(t_n + 0.5 * h, x_k3, stFcn, wsFcn, params, pureParams, combinedParams);

        % Stage 4
        x_k4 = x_curr + h * k3;
        k4 = vehicleDerivative(t_n + h, x_k4, stFcn, wsFcn, params, pureParams, combinedParams);

        % State update
        x_curr = x_curr + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        stateHistory(i + 1, :) = x_curr';
    end
end

% ===========================================================================
% Local derivative function: 6-state continuous dynamics
% ===========================================================================
function xdot = vehicleDerivative(t, x, steeringFcn, wheelSpeedFcn, params, pureParams, combinedParams)
    psi = x(3);
    vx  = x(4);
    vy  = x(5);
    r   = x(6);

    delta = steeringFcn(t);
    omega = wheelSpeedFcn(t);

    % 1. Global pose kinematics
    dot_X   = vx * cos(psi) - vy * sin(psi);
    dot_Y   = vx * sin(psi) + vy * cos(psi);
    dot_psi = r;

    % 2. Phase 3F closed-loop vehicle dynamic derivatives
    sdot_3dof = closedLoopVehicleDerivative([vx; vy; r], delta, omega, params, pureParams, combinedParams);

    % Complete 6x1 derivative
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
function fcn = wrapInput(inputVal, name, expectedElements)
    if isa(inputVal, 'function_handle')
        try
            testVal = inputVal(0.0);
        catch ME
            error('runVehicleSimulation:invalidFunctionHandle', ...
                  'Function handle %s failed at t=0: %s', name, ME.message);
        end
        if ~isnumeric(testVal) || ~isreal(testVal)
            error('runVehicleSimulation:invalidInput', ...
                  'Function handle %s must return real numeric values.', name);
        end
        fcn = inputVal;
    elseif isnumeric(inputVal) && isreal(inputVal)
        val = inputVal;
        fcn = @(t) val;
    else
        error('runVehicleSimulation:invalidInputType', ...
              '%s must be a real numeric constant or a function handle @(t).', name);
    end
end
