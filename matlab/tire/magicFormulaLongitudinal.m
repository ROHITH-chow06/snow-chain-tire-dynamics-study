function Fx = magicFormulaLongitudinal(kappa, Fz, B, C, D, E)
% MAGICFORMULALONGITUDINAL Calculate pure longitudinal tire force using Magic Formula.
%
%   Fx = magicFormulaLongitudinal(kappa, Fz, B, C, D, E)
%
%   Calculates the steady-state longitudinal tire force (Fx) generated at
%   the tire-road contact patch using a simplified constant-coefficient
%   Pacejka Magic Formula formulation:
%
%       Fx = Fz .* D .* sin(C .* atan(B .* kappa - E .* (B .* kappa - atan(B .* kappa))))
%
%   Inputs:
%       kappa - Longitudinal slip ratio [-] (numeric, real array or scalar)
%       Fz    - Vertical tire normal load [N] (numeric, real, positive, array or scalar)
%       B     - Stiffness factor [-] (numeric, real, scalar)
%       C     - Shape factor [-] (numeric, real, scalar)
%       D     - Peak factor [-] (numeric, real, scalar)
%       E     - Curvature factor [-] (numeric, real, scalar)
%
%   Output:
%       Fx    - Longitudinal tire force [N]
%
%   Small-Slip Slope:
%       The initial slope at zero slip is given by:
%       dFx/dkappa (at kappa = 0) = B * C * D * Fz
%
%   Sign Convention:
%       kappa > 0 -> Driving / tractive force (Fx > 0)
%       kappa < 0 -> Braking / retarding force (Fx < 0)
%       kappa = 0 -> Zero force (Fx = 0)
%
%   Assumptions & Limitations:
%       - Constant B, C, D, E coefficients (load-dependent variations not included).
%       - Pure longitudinal motion (zero lateral slip angle, alpha = 0).
%       - Quasi-steady contact patch conditions without dynamic relaxation length.

    validateattributes(kappa, {'numeric'}, {'real', 'nonempty'}, mfilename, 'kappa', 1);
    validateattributes(Fz, {'numeric'}, {'real', 'positive', 'nonempty'}, mfilename, 'Fz', 2);
    validateattributes(B, {'numeric'}, {'real', 'scalar', 'nonempty'}, mfilename, 'B', 3);
    validateattributes(C, {'numeric'}, {'real', 'scalar', 'nonempty'}, mfilename, 'C', 4);
    validateattributes(D, {'numeric'}, {'real', 'scalar', 'nonempty'}, mfilename, 'D', 5);
    validateattributes(E, {'numeric'}, {'real', 'scalar', 'nonempty'}, mfilename, 'E', 6);

    % Dimension compatibility check between kappa and Fz
    if ~isscalar(kappa) && ~isscalar(Fz) && ~isequal(size(kappa), size(Fz))
        error('magicFormulaLongitudinal:dimensionMismatch', ...
            'Inputs kappa and Fz must have identical dimensions, or one input must be scalar.');
    end

    % Simplified Magic Formula calculation
    B_kappa = B .* kappa;
    inner_term = B_kappa - E .* (B_kappa - atan(B_kappa));
    Fx = Fz .* D .* sin(C .* atan(inner_term));
end
