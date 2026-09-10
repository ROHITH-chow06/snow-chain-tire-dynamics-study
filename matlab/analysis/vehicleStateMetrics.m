function metrics = vehicleStateMetrics(time, stateHistory, params)
% VEHICLESTATEMETRICS Extracts and computes physical metrics from simulation state history.
%
%   metrics = vehicleStateMetrics(time, stateHistory, params)
%
%   Calculates physically meaningful vehicle-level quantities derived directly
%   from the state history. This is an analysis characterization of the
%   implemented mathematical model, not an experimental measurement.
%
%   Inputs:
%       time         - N x 1 simulation time vector [s]
%       stateHistory - N x 6 vehicle state history [X, Y, psi, vx, vy, r]
%       params       - Rover parameter struct (included for API consistency)
%
%   Outputs:
%       metrics      - Struct containing time histories of:
%                      .X, .Y, .psi, .vx, .vy, .r - Base states
%                      .speed                     - Ground speed magnitude [m/s]
%                      .beta                      - Sideslip angle [rad]
%                      .ax                        - Body longitudinal acceleration [m/s^2]
%                      .ay                        - Body lateral acceleration [m/s^2]
%                      .yawAcceleration           - Yaw acceleration [rad/s^2]
%                      .curvature                 - Path curvature [1/m]

    % Validate inputs
    validateattributes(time, {'numeric'}, {'vector', 'real'}, mfilename, 'time');
    validateattributes(stateHistory, {'numeric'}, {'2d', 'real', 'ncols', 6}, mfilename, 'stateHistory');
    if length(time) ~= size(stateHistory, 1)
        error('vehicleStateMetrics:dimensionMismatch', 'time vector length must match stateHistory rows.');
    end

    % Extract base states
    metrics.X   = stateHistory(:, 1);
    metrics.Y   = stateHistory(:, 2);
    metrics.psi = stateHistory(:, 3);
    metrics.vx  = stateHistory(:, 4);
    metrics.vy  = stateHistory(:, 5);
    metrics.r   = stateHistory(:, 6);

    % Derived state metrics
    metrics.speed = sqrt(metrics.vx.^2 + metrics.vy.^2);
    metrics.beta  = atan2(metrics.vy, metrics.vx);

    % Numerical differentiation for accelerations
    % Handle potential non-uniform time steps (e.g. from ode45)
    if length(time) > 1
        % Compute time derivative of state components
        dvx_dt = gradient(metrics.vx, time);
        dvy_dt = gradient(metrics.vy, time);
        dr_dt  = gradient(metrics.r, time);
    else
        % Fallback for single time point (edge case)
        dvx_dt = 0;
        dvy_dt = 0;
        dr_dt  = 0;
    end

    % Body frame physical accelerations (including Coriolis/centripetal effects)
    % Consistent with Phase 3F dynamics definition:
    % ax = dot(vx) - vy * r
    % ay = dot(vy) + vx * r
    metrics.ax = dvx_dt - metrics.vy .* metrics.r;
    metrics.ay = dvy_dt + metrics.vx .* metrics.r;
    metrics.yawAcceleration = dr_dt;

    % Path curvature = r / V. Protected against division by zero.
    metrics.curvature = zeros(size(time));
    speed_threshold = 1e-8;
    idx_nonzero = metrics.speed > speed_threshold;
    metrics.curvature(idx_nonzero) = metrics.r(idx_nonzero) ./ metrics.speed(idx_nonzero);

end
