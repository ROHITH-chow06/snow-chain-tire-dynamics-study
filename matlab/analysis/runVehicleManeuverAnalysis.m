function results = runVehicleManeuverAnalysis(maneuverType, params, varargin)
% RUNVEHICLEMANEUVERANALYSIS Executes standardized maneuvers and computes analysis metrics.
%
%   results = runVehicleManeuverAnalysis(maneuverType, params, pure_params, combined_params, sim_options)
%
%   High-level driver that defines deterministic maneuvers, runs the Phase 3H
%   trajectory integration layer, and computes Phase 3I analysis metrics.
%   This is an analysis characterization of the model, not an experimental measurement.
%
%   Supported Maneuvers:
%       'StraightAcceleration'          - Zero steering, increasing wheel speed
%       'StraightBraking'               - Zero steering, decreasing wheel speed
%       'ConstantCornering'             - Step steering, constant wheel speed
%       'SteeringSweep'                 - Sinusoidal steering excitation
%       'CombinedBrakingCornering'      - Step steering + braking
%       'CombinedAccelerationCornering' - Step steering + acceleration
%
%   Inputs:
%       maneuverType    - String name of the maneuver
%       params          - Rover parameter struct
%       varargin        - Optional: pure_params, combined_params, sim_options
%
%   Outputs:
%       results         - Struct containing:
%                         .time         - Simulation time
%                         .state        - State history
%                         .diagnostics  - Phase 3H diagnostics
%                         .stateMetrics - vehicleStateMetrics output
%                         .forceMetrics - tireForceMetrics output
%                         .handling     - vehicleHandlingMetrics output

    % Handle optional arguments
    if nargin > 2 && ~isempty(varargin{1}), pure_params = varargin{1}; else, pure_params = []; end
    if nargin > 3 && ~isempty(varargin{2}), combined_params = varargin{2}; else, combined_params = []; end
    if nargin > 4 && ~isempty(varargin{3}), sim_options = varargin{3}; else, sim_options = struct(); end
    
    % Default setup
    R = params.R;
    tSpan = [0, 3.0];
    x0 = [0.0; 0.0; 0.0; 1.0; 0.0; 0.0]; % [X, Y, psi, vx, vy, r]
    
    switch maneuverType
        case 'StraightAcceleration'
            % Start at 1 m/s, accelerate
            x0(4) = 1.0;
            deltaFcn = 0.0;
            omegaFcn = @(t) ((1.0 + 1.0 * t) / R) * ones(4, 1);
            
        case 'StraightBraking'
            % Start at 3 m/s, brake
            x0(4) = 3.0;
            deltaFcn = 0.0;
            omegaFcn = @(t) (max(0.1, 3.0 - 1.0 * t) / R) * ones(4, 1);
            
        case 'ConstantCornering'
            % Start at 2 m/s, step steering
            x0(4) = 2.0;
            deltaFcn = @(t) 0.1 * (t > 0.5); % 0.1 rad step at t=0.5
            omegaFcn = (2.0 / R) * ones(4, 1);
            
        case 'SteeringSweep'
            % Start at 2 m/s, sinusoidal steering
            x0(4) = 2.0;
            freq = 0.5; % Hz
            deltaFcn = @(t) 0.05 * sin(2 * pi * freq * t);
            omegaFcn = (2.0 / R) * ones(4, 1);
            
        case 'CombinedBrakingCornering'
            % Start at 3 m/s, brake and turn simultaneously
            x0(4) = 3.0;
            deltaFcn = @(t) 0.1 * (t > 0.5);
            omegaFcn = @(t) (max(0.1, 3.0 - 0.5 * t) / R) * ones(4, 1);
            
        case 'CombinedAccelerationCornering'
            % Start at 1 m/s, accelerate and turn simultaneously
            x0(4) = 1.0;
            deltaFcn = @(t) 0.1 * (t > 0.5);
            omegaFcn = @(t) ((1.0 + 0.5 * t) / R) * ones(4, 1);
            
        otherwise
            error('runVehicleManeuverAnalysis:unknownManeuver', 'Unknown maneuver type: %s', maneuverType);
    end

    % Execute existing Phase 3H trajectory integration layer
    [time, state, diagnostics] = simulateVehicleTrajectory(tSpan, x0, deltaFcn, omegaFcn, ...
                                                           params, pure_params, combined_params, sim_options);
                                                           
    % Phase 3I Analysis Layer
    stateMetrics = vehicleStateMetrics(time, state, params);
    forceMetrics = tireForceMetrics(diagnostics, params);
    
    % Reconstruct steering array for handling metrics calculation
    % (If deltaFcn is a handle, evaluate it; if scalar, expand it)
    N = length(time);
    delta_array = zeros(N, 4);
    for k = 1:N
        if isa(deltaFcn, 'function_handle')
            d_val = deltaFcn(time(k));
        else
            d_val = deltaFcn;
        end
        if numel(d_val) == 1
            delta_array(k, :) = d_val * ones(1, 4);
        else
            delta_array(k, :) = d_val(:)';
        end
    end
    
    handling = vehicleHandlingMetrics(stateMetrics, forceMetrics, delta_array, time, params);
    
    % Package results
    results.time         = time;
    results.state        = state;
    results.diagnostics  = diagnostics;
    results.stateMetrics = stateMetrics;
    results.forceMetrics = forceMetrics;
    results.handling     = handling;

end
