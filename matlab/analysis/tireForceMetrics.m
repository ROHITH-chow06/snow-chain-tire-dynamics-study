function metrics = tireForceMetrics(diagnostics, params)
% TIREFORCEMETRICS Extracts and computes diagnostic tire force metrics.
%
%   metrics = tireForceMetrics(diagnostics, params)
%
%   Computes normalized tire-force utilization metrics derived from the implemented
%   combined-slip force model. This is a diagnostic characterization of the
%   model's force utilization, not a claimed physical friction-circle law or
%   experimentally validated friction boundaries.
%
%   Inputs:
%       diagnostics - Diagnostics struct from simulateVehicleTrajectory
%       params      - Rover parameter struct (included for API consistency)
%
%   Outputs:
%       metrics     - Struct containing time histories of:
%                     .mu_x        - N x 4 normalized longitudinal force (abs(Fx/Fz))
%                     .mu_y        - N x 4 normalized lateral force (abs(Fy/Fz))
%                     .mu_combined - N x 4 combined utilization metric
%                     .total_Fx    - N x 1 total longitudinal body force [N]
%                     .total_Fy    - N x 1 total lateral body force [N]
%                     .total_Mz    - N x 1 total body yaw moment [N*m]

    % Validate inputs
    if ~isstruct(diagnostics) || ~isfield(diagnostics, 'Fx_wheel') || ~isfield(diagnostics, 'Fz_wheel')
        error('tireForceMetrics:invalidInput', 'Input must be a valid diagnostics structure.');
    end

    % Strict validation of normal load to ensure physical validity
    if any(~isfinite(diagnostics.Fz_wheel(:))) || any(~isreal(diagnostics.Fz_wheel(:)))
        error('tireForceMetrics:invalidNormalLoad', 'Normal load must be finite and real.');
    end
    if any(diagnostics.Fz_wheel(:) <= 0)
        error('tireForceMetrics:invalidNormalLoad', 'Normal load must be strictly positive for all wheels.');
    end
    
    Fz_valid = diagnostics.Fz_wheel;

    % Normalized force-utilization diagnostics
    metrics.mu_x = abs(diagnostics.Fx_wheel ./ Fz_valid);
    metrics.mu_y = abs(diagnostics.Fy_wheel ./ Fz_valid);

    % Diagnostic combined utilization (dimensionless metric)
    metrics.mu_combined = sqrt(metrics.mu_x.^2 + metrics.mu_y.^2);

    % Vehicle-level totals (pass through from diagnostics, which uses the Phase 3F convention)
    metrics.total_Fx = diagnostics.Fx_total;
    metrics.total_Fy = diagnostics.Fy_total;
    metrics.total_Mz = diagnostics.Mz_total;
end
