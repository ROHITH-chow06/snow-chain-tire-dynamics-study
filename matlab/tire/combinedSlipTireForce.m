function [Fx, Fy, Gxa, Gyk] = combinedSlipTireForce(kappa, alpha, Fz, pure_params, combined_params)
% COMBINEDSLIPTIREFORCE Computes combined-slip tire forces.
%
%   [Fx, Fy, Gxa, Gyk] = combinedSlipTireForce(kappa, alpha, Fz, pure_params, combined_params)
%   calculates the longitudinal and lateral tire forces under combined slip
%   using the reduced PAC2002-style cosine weighting formulation.
%
%   Inputs:
%       kappa           - Longitudinal slip ratio
%       alpha           - Lateral slip angle (rad)
%       Fz              - Normal load (N)
%       pure_params     - Struct with fields: Bx, Cx, Dx, Ex, By, Cy, Dy, Ey
%       combined_params - Struct with fields: 
%                         Bx_alpha, Cx_alpha, Ex_alpha, SHx_alpha,
%                         By_kappa, Cy_kappa, Ey_kappa, SHy_kappa
%
%   Outputs:
%       Fx  - Combined longitudinal force (N)
%       Fy  - Combined lateral force (N)
%       Gxa - Longitudinal weighting factor
%       Gyk - Lateral weighting factor

    % Validate inputs
    if nargin < 3
        error('combinedSlipTireForce:notEnoughInputs', 'Requires at least kappa, alpha, and Fz.');
    end
    % Collect sizes of non-scalar inputs and check compatibility
    sizes = {};
    if ~isscalar(kappa), sizes{end+1} = size(kappa); end
    if ~isscalar(alpha), sizes{end+1} = size(alpha); end
    if ~isscalar(Fz),    sizes{end+1} = size(Fz);    end
    for i = 2:numel(sizes)
        if any(sizes{i} ~= sizes{1})
            error('combinedSlipTireForce:dimensionMismatch', 'Inputs kappa, alpha, and Fz must have compatible dimensions.');
        end
    end
    if any(Fz(:) <= 0)
        error('combinedSlipTireForce:invalidNormalLoad', 'Normal load Fz must be positive.');
    end

    % Default pure-slip parameters if not provided
    if nargin < 4 || isempty(pure_params)
        pure_params = struct('Bx', 10.0, 'Cx', 1.90, 'Dx', 1.00, 'Ex', 0.97, ...
                             'By', 10.0, 'Cy', 1.30, 'Dy', 1.00, 'Ey', -1.00);
    end

    % Default combined-slip parameters if not provided
    if nargin < 5 || isempty(combined_params)
        combined_params = struct('Bx_alpha', 10.0, 'Cx_alpha', 1.00, 'Ex_alpha', -1.00, 'SHx_alpha', 0.0, ...
                                 'By_kappa', 10.0, 'Cy_kappa', 1.00, 'Ey_kappa', -1.00, 'SHy_kappa', 0.0);
    end

    % Calculate pure-slip constituent forces
    Fx0 = magicFormulaLongitudinal(kappa, Fz, pure_params.Bx, pure_params.Cx, pure_params.Dx, pure_params.Ex);
    Fy0 = magicFormulaLateral(alpha, Fz, pure_params.By, pure_params.Cy, pure_params.Dy, pure_params.Ey);

    % Calculate weighting functions
    Gxa = combinedSlipWeighting(alpha, combined_params.Bx_alpha, combined_params.Cx_alpha, ...
                                combined_params.Ex_alpha, combined_params.SHx_alpha);
                            
    Gyk = combinedSlipWeighting(kappa, combined_params.By_kappa, combined_params.Cy_kappa, ...
                                combined_params.Ey_kappa, combined_params.SHy_kappa);

    % Calculate combined forces
    Fx = Gxa .* Fx0;
    Fy = Gyk .* Fy0;
end
