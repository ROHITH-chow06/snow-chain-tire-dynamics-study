function comparison = runRoadConditionComparison(maneuverType, params, varargin)
% RUNROADCONDITIONCOMPARISON Executes a comparative maneuver analysis across road surface conditions.
%
%   comparison = runRoadConditionComparison(maneuverType, params)
%   comparison = runRoadConditionComparison(maneuverType, params, sim_options)
%
%   This function sits above Phase 3I. It takes a single vehicle definition 
%   (params) and a specific maneuver, then deterministically runs it under 
%   'Dry', 'Wet', and 'Snow' conditions using structurally identical wheel/steering 
%   commands.
%
%   Inputs:
%       maneuverType - String name of the maneuver (e.g. 'StraightAcceleration')
%       params       - Rover parameter struct (must remain invariant across runs)
%       sim_options  - (Optional) Simulation solver settings struct
%
%   Outputs:
%       comparison   - Struct containing the Phase 3I analysis results for each
%                      surface condition, structured as:
%                         comparison.Dry
%                         comparison.Wet
%                         comparison.Snow
%
%   Condition constraints enforced:
%       - Same vehicle parameters
%       - Same initial state (via runVehicleManeuverAnalysis deterministic internal definition)
%       - Same time step / solver settings
%       - Same exact wheel-speed and steering command histories

    if nargin > 2 && ~isempty(varargin{1})
        sim_options = varargin{1};
    else
        sim_options = struct();
    end

    conditions = {'Dry', 'Wet', 'Snow'};
    
    for i = 1:length(conditions)
        condName = conditions{i};
        
        % 1. Get mathematically representative condition parameters
        condParams = roadSurfaceCondition(condName);
        
        % 2. Run the deterministic Phase 3I maneuver analysis
        % Passing condition parameters directly into the established stack
        res = runVehicleManeuverAnalysis(maneuverType, params, ...
                                         condParams.pure_params, ...
                                         condParams.combined_params, ...
                                         sim_options);
                                         
        % 3. Store result in output struct
        comparison.(condName) = res;
    end
end
