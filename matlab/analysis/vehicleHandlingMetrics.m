function handling = vehicleHandlingMetrics(stateMetrics, forceMetrics, delta, time, params)
% VEHICLEHANDLINGMETRICS Computes handling metrics and summary scalars.
%
%   handling = vehicleHandlingMetrics(stateMetrics, forceMetrics, delta, time, params)
%
%   Calculates time-series handling metrics and maneuver-level summary scalars.
%   These are model-characterization quantities, not experimental vehicle certification.
%
%   Inputs:
%       stateMetrics - Struct from vehicleStateMetrics
%       forceMetrics - Struct from tireForceMetrics
%       delta        - N x 4 or N x 1 steering angle vector [rad]
%       time         - N x 1 simulation time vector [s]
%       params       - Rover parameter struct (included for API consistency)
%
%   Outputs:
%       handling     - Struct containing:
%                      .yawRateGain      - N x 1 yaw-rate gain (r / delta) [1/s]
%                      .lateralAccelGain - N x 1 lateral accel gain (ay / delta) [m/s^2 / rad]
%                      .peak_speed       - Scalar max speed
%                      .peak_ax          - Scalar max absolute ax
%                      .peak_ay          - Scalar max absolute ay
%                      .peak_abs_r       - Scalar max absolute yaw rate
%                      .peak_abs_beta    - Scalar max absolute sideslip
%                      .peak_tire_util   - Scalar max combined tire utilization
%                      .peak_Fx          - Scalar max absolute total longitudinal force
%                      .peak_Fy          - Scalar max absolute total lateral force
%                      .final_X          - Scalar final X position
%                      .final_Y          - Scalar final Y position
%                      .final_psi        - Scalar final heading
%                      .final_speed      - Scalar final speed

    % Validate inputs
    if ~isstruct(stateMetrics) || ~isstruct(forceMetrics)
        error('vehicleHandlingMetrics:invalidInput', 'stateMetrics and forceMetrics must be structs.');
    end

    % Standardize delta to N x 1 (using front-left as reference for handling gain)
    % Assuming Ackermann or symmetric parallel steering. 
    if size(delta, 2) == 4
        delta_ref = delta(:, 1);
    else
        delta_ref = delta;
    end
    
    % Time-series Handling Metrics
    % Guard against division by zero (steering near zero)
    steer_threshold = 1e-4; % rad
    idx_steer = abs(delta_ref) > steer_threshold;
    
    handling.yawRateGain = zeros(size(time));
    handling.lateralAccelGain = zeros(size(time));
    
    handling.yawRateGain(idx_steer) = stateMetrics.r(idx_steer) ./ delta_ref(idx_steer);
    handling.lateralAccelGain(idx_steer) = stateMetrics.ay(idx_steer) ./ delta_ref(idx_steer);

    % Curvature is already in stateMetrics, beta is in stateMetrics
    
    % Maneuver-level Summary Metrics (Scalars)
    handling.peak_speed    = max(stateMetrics.speed);
    handling.peak_ax       = max(abs(stateMetrics.ax));
    handling.peak_ay       = max(abs(stateMetrics.ay));
    handling.peak_abs_r    = max(abs(stateMetrics.r));
    handling.peak_abs_beta = max(abs(stateMetrics.beta));
    
    % Peak tire utilization across all wheels and all time steps
    handling.peak_tire_util = max(forceMetrics.mu_combined(:));
    
    handling.peak_Fx = max(abs(forceMetrics.total_Fx));
    handling.peak_Fy = max(abs(forceMetrics.total_Fy));
    
    % Final states
    handling.final_X     = stateMetrics.X(end);
    handling.final_Y     = stateMetrics.Y(end);
    handling.final_psi   = stateMetrics.psi(end);
    handling.final_speed = stateMetrics.speed(end);

end
