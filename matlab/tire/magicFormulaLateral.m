function Fy = magicFormulaLateral(alpha, Fz, By, Cy, Dy, Ey)
% MAGICFORMULALATERAL Calculate pure lateral tire force using Magic Formula.
%
%   Fy = magicFormulaLateral(alpha, Fz, By, Cy, Dy, Ey)
%
%   Calculates the steady-state lateral tire force (Fy) generated at the
%   tire-road contact patch using a simplified constant-coefficient Pacejka
%   Magic Formula formulation under the project's signed slip-angle convention:
%
%       Fy_MF = Fz .* Dy .* sin(Cy .* atan(By .* alpha - Ey .* (By .* alpha - atan(By .* alpha))))
%       Fy    = -Fy_MF
%
%   Inputs:
%       alpha - Wheel velocity / slip angle [rad] (numeric, real array or scalar)
%       Fz    - Vertical tire normal load [N] (numeric, real, positive, array or scalar)
%       By    - Lateral stiffness factor [-] (numeric, real, scalar)
%       Cy    - Lateral shape factor [-] (numeric, real, scalar)
%       Dy    - Lateral peak factor [-] (numeric, real, scalar)
%       Ey    - Lateral curvature factor [-] (numeric, real, scalar)
%
%   Output:
%       Fy    - Lateral tire force [N] in the wheel coordinate frame
%
%   Sign Convention & Project Definition:
%       - alpha is defined as the signed wheel velocity angle: alpha = atan2(vy_wheel, vx_wheel).
%       - For small slip angles, the tire generates a restoring lateral force that opposes
%         lateral velocity: Fy ≈ -K_alpha * alpha, where K_alpha = By * Cy * Dy * Fz.
%       - alpha > 0 (vy_wheel > 0, leftward velocity) -> Fy < 0 (rightward restoring force).
%       - alpha < 0 (vy_wheel < 0, rightward velocity) -> Fy > 0 (leftward restoring force).
%       - alpha = 0 -> Fy = 0.
%
%   Assumptions & Limitations:
%       - Constant By, Cy, Dy, Ey coefficients (load-dependent parameter variations not included).
%       - Pure lateral motion (zero longitudinal slip, kappa = 0; no combined slip).
%       - Zero camber angle (gamma = 0) and zero horizontal/vertical shifts (Sh = 0, Sv = 0).
%       - Quasi-steady contact patch conditions without dynamic relaxation length or aligning moment.

    validateattributes(alpha, {'numeric'}, {'real', 'nonempty'}, mfilename, 'alpha', 1);
    validateattributes(Fz,    {'numeric'}, {'real', 'positive', 'nonempty'}, mfilename, 'Fz', 2);
    validateattributes(By,    {'numeric'}, {'real', 'scalar', 'nonempty'}, mfilename, 'By', 3);
    validateattributes(Cy,    {'numeric'}, {'real', 'scalar', 'nonempty'}, mfilename, 'Cy', 4);
    validateattributes(Dy,    {'numeric'}, {'real', 'scalar', 'nonempty'}, mfilename, 'Dy', 5);
    validateattributes(Ey,    {'numeric'}, {'real', 'scalar', 'nonempty'}, mfilename, 'Ey', 6);

    % Dimension compatibility check between alpha and Fz
    if ~isscalar(alpha) && ~isscalar(Fz) && ~isequal(size(alpha), size(Fz))
        error('magicFormulaLateral:dimensionMismatch', ...
            'Inputs alpha and Fz must have identical dimensions, or one input must be scalar.');
    end

    % Simplified pure-lateral Magic Formula calculation
    B_alpha = By .* alpha;
    inner_term = B_alpha - Ey .* (B_alpha - atan(B_alpha));
    Fy_MF = Fz .* Dy .* sin(Cy .* atan(inner_term));
    Fy = -Fy_MF;
end
