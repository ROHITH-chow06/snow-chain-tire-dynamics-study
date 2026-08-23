function alpha = wheelSlipAngle(vy_wheel, vx_wheel, v_threshold)
% WHEELSLIPANGLE Calculate signed wheel velocity slip angle.
%
%   alpha = wheelSlipAngle(vy_wheel, vx_wheel)
%   alpha = wheelSlipAngle(vy_wheel, vx_wheel, v_threshold)
%
%   Calculates the signed wheel velocity / slip angle (alpha) in the wheel
%   coordinate frame according to the explicitly defined project convention:
%
%       alpha = atan2(vy_wheel, vx_wheel)
%
%   For speeds below the low-speed threshold (sqrt(vx^2 + vy^2) < v_threshold),
%   the slip angle is set to 0.0 rad to maintain numerical regularity and
%   avoid ill-conditioned angle fluctuations at standstill.
%
%   Inputs:
%       vy_wheel    - Lateral velocity in wheel coordinate frame [m/s] (numeric, real)
%       vx_wheel    - Longitudinal velocity in wheel coordinate frame [m/s] (numeric, real)
%       v_threshold - (Optional) Low-speed magnitude threshold [m/s].
%                     Default: 1e-4 m/s.
%
%   Output:
%       alpha       - Signed slip angle [rad] (bounded in [-pi, pi])
%
%   Project Convention & Sign Definition:
%       - Wheel frame: +x forward along wheel heading, +y leftward perpendicular to wheel plane.
%       - alpha > 0 : Wheel velocity vector has a positive leftward lateral component (vy_wheel > 0).
%       - alpha < 0 : Wheel velocity vector has a negative rightward lateral component (vy_wheel < 0).
%       - For a rover traveling forward (vx_body > 0, vy_body = 0) with front wheels steered
%         to the left (delta > 0), the relative ground velocity in the wheel frame is
%         vy_wheel = -vx_body * sin(delta) < 0, resulting in alpha < 0.
%
%   Literature Context:
%       This definition is the project's explicitly adopted kinematic wheel velocity
%       angle convention. Literature sources define tire slip angles with varying signs
%       (e.g., SAE J670, ISO 8855, Pacejka). The lateral tire-force constitutive relation
%       will be formulated and connected to this kinematic angle in Phase 3B.
%
%   Numerical Regularization:
%       At true standstill (vx_wheel = 0, vy_wheel = 0) and for velocity magnitudes
%       below v_threshold, alpha evaluates strictly to 0.0 without producing NaN or Inf.

    if nargin < 3 || isempty(v_threshold)
        v_threshold = 1e-4;
    end

    validateattributes(vy_wheel, {'numeric'}, {'real', 'nonempty'}, mfilename, 'vy_wheel', 1);
    validateattributes(vx_wheel, {'numeric'}, {'real', 'nonempty'}, mfilename, 'vx_wheel', 2);
    validateattributes(v_threshold, {'numeric'}, {'real', 'nonnegative', 'scalar'}, mfilename, 'v_threshold', 3);

    % Dimension compatibility
    if ~isscalar(vy_wheel) && ~isscalar(vx_wheel) && ~isequal(size(vy_wheel), size(vx_wheel))
        error('wheelSlipAngle:dimensionMismatch', ...
            'Inputs vy_wheel and vx_wheel must have identical dimensions, or one input must be scalar.');
    end

    % Element-wise computation
    speed = hypot(vx_wheel, vy_wheel);
    alpha = atan2(vy_wheel, vx_wheel);

    % Regularize near standstill / below threshold to zero
    low_speed_mask = (speed < v_threshold);
    alpha(low_speed_mask) = 0.0;
end
