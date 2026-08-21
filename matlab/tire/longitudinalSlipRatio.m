function kappa = longitudinalSlipRatio(R, omega, Vx, epsilon)
% LONGITUDINALSLIPRATIO Calculate longitudinal tire slip ratio.
%
%   kappa = longitudinalSlipRatio(R, omega, Vx)
%   kappa = longitudinalSlipRatio(R, omega, Vx, epsilon)
%
%   Calculates the longitudinal slip ratio (kappa) of a rolling wheel
%   relative to the vehicle longitudinal velocity according to:
%
%       kappa = (R .* omega - Vx) ./ max(abs(Vx), epsilon)
%
%   Inputs:
%       R       - Effective wheel rolling radius [m] (numeric, positive)
%       omega   - Wheel rotational angular velocity [rad/s] (numeric, real)
%       Vx      - Vehicle longitudinal velocity [m/s] (numeric, real)
%       epsilon - (Optional) Low-speed regularization threshold [m/s].
%                 Default: 0.1 m/s to prevent division by zero near standstill.
%
%   Output:
%       kappa   - Longitudinal slip ratio [-]
%
%   Sign Convention:
%       kappa == 0 : Pure rolling (R * omega == Vx)
%       kappa > 0  : Driving / acceleration slip (wheel spins faster than vehicle)
%       kappa < 0  : Braking slip (wheel spins slower than vehicle, locked wheel -> -1)
%
%   Limitations:
%       Conventional kinematic slip formulations become ill-conditioned near
%       zero forward velocity (Vx -> 0). The parameter epsilon prevents numerical
%       singularity (division by zero) but does not substitute for a dynamic
%       low-speed or standstill tire deflection model.

    if nargin < 4 || isempty(epsilon)
        epsilon = 0.1;
    end

    validateattributes(R, {'numeric'}, {'real', 'positive', 'nonempty'}, mfilename, 'R', 1);
    validateattributes(omega, {'numeric'}, {'real', 'nonempty'}, mfilename, 'omega', 2);
    validateattributes(Vx, {'numeric'}, {'real', 'nonempty'}, mfilename, 'Vx', 3);
    validateattributes(epsilon, {'numeric'}, {'real', 'positive', 'scalar', 'nonempty'}, mfilename, 'epsilon', 4);

    denominator = max(abs(Vx), epsilon);
    kappa = (R .* omega - Vx) ./ denominator;
end
