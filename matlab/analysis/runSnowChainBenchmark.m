function benchmark = runSnowChainBenchmark(params, varargin)
% RUNSNOWCHAINBENCHMARK Executes a comparative maneuver analysis between a baseline normal tire on snow and the simplified SnowChain benchmark.
%
%   benchmark = runSnowChainBenchmark(params)
%   benchmark = runSnowChainBenchmark(params, sim_options)
%
%   This function serves as the primary analysis script for the Phase 3K
%   Snow-Chain Benchmark. It runs straight acceleration, straight braking, 
%   and constant-radius cornering maneuvers.
%
%   Inputs:
%       params       - Four-wheel vehicle parameter struct 
%       sim_options  - (Optional) Simulation solver settings struct
%
%   Outputs:
%       benchmark    - Struct containing the comparative results:
%                         benchmark.StraightAcceleration.Snow
%                         benchmark.StraightAcceleration.SnowChain
%                         benchmark.StraightBraking.Snow
%                         ...

    if nargin > 1 && ~isempty(varargin{1})
        sim_options = varargin{1};
    else
        sim_options = struct();
    end

    % 1. Get conditions
    cond_snow = roadSurfaceCondition('Snow');
    cond_chain = snowChainCondition();
    
    conditions = {cond_snow, cond_chain};
    cond_names = {'Snow', 'SnowChain'};
    maneuvers = {'StraightAcceleration', 'StraightBraking', 'ConstantCornering'};
    
    benchmark = struct();
    
    for m = 1:length(maneuvers)
        maneuverType = maneuvers{m};
        
        for c = 1:length(conditions)
            cond = conditions{c};
            name = cond_names{c};
            
            res = runVehicleManeuverAnalysis(maneuverType, params, ...
                                             cond.pure_params, ...
                                             cond.combined_params, ...
                                             sim_options);
            
            benchmark.(maneuverType).(name) = res;
        end
    end
end
