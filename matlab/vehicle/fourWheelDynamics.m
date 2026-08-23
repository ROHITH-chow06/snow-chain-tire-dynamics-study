function [state_deriv, dynamics] = fourWheelDynamics(vx, vy, r, delta, Fx_wheel, Fy_wheel, params)
% FOURWHEELDYNAMICS Compute 3-DOF planar rigid-body vehicle dynamics.
%
%   state_deriv = fourWheelDynamics(vx, vy, r, delta, Fx_wheel, Fy_wheel)
%   state_deriv = fourWheelDynamics(vx, vy, r, delta, Fx_wheel, Fy_wheel, params)
%   [state_deriv, dynamics] = fourWheelDynamics(vx, vy, r, delta, Fx_wheel, Fy_wheel, params)
%
%   Computes the 3-DOF planar equations of motion for a four-wheel rover:
%       m * (dot(vx) - vy * r) = sum(Fx_body)
%       m * (dot(vy) + vx * r) = sum(Fy_body)
%       Iz * dot(r)            = sum(Mz)
%
%   Wheel ordering:
%       Index 1: FL (Front-Left)
%       Index 2: FR (Front-Right)
%       Index 3: RL (Rear-Left)
%       Index 4: RR (Rear-Right)
%
%   Inputs:
%       vx          - Longitudinal vehicle body velocity [m/s] (scalar)
%       vy          - Lateral vehicle body velocity [m/s] (scalar)
%       r           - Vehicle yaw rate [rad/s] (scalar, positive CCW)
%       delta       - Front steering angle [rad] (scalar, or 4x1 vector [delta_FL; delta_FR; delta_RL; delta_RR])
%       Fx_wheel    - 4x1 or 1x4 vector of wheel-frame longitudinal tire forces [N]
%       Fy_wheel    - 4x1 or 1x4 vector of wheel-frame lateral tire forces [N]
%       params      - (Optional) Rover parameter struct from roverParameters()
%
%   Outputs:
%       state_deriv - 3x1 vector of state time derivatives:
%                     [dot_vx; dot_vy; dot_r]  ([m/s^2; m/s^2; rad/s^2])
%       dynamics    - Struct containing transformed forces and moments:
%                     .Fx_body   - 4x1 individual body-frame longitudinal forces [N]
%                     .Fy_body   - 4x1 individual body-frame lateral forces [N]
%                     .Fx_total  - Total longitudinal force on vehicle body [N]
%                     .Fy_total  - Total lateral force on vehicle body [N]
%                     .Mz_i      - 4x1 individual wheel yaw moments about CG [N*m]
%                     .Mz_total  - Total yaw moment about CG [N*m]
%                     .dot_vx    - Longitudinal acceleration dot(vx) [m/s^2]
%                     .dot_vy    - Lateral acceleration dot(vy) [m/s^2]
%                     .dot_r     - Yaw acceleration dot(r) [rad/s^2]

    if nargin < 7 || isempty(params)
        params = roverParameters();
    end

    % Input validation
    validateattributes(vx, {'numeric'}, {'real', 'scalar'}, mfilename, 'vx', 1);
    validateattributes(vy, {'numeric'}, {'real', 'scalar'}, mfilename, 'vy', 2);
    validateattributes(r,  {'numeric'}, {'real', 'scalar'}, mfilename, 'r', 3);
    validateattributes(delta, {'numeric'}, {'real', 'vector'}, mfilename, 'delta', 4);
    validateattributes(Fx_wheel, {'numeric'}, {'real', 'vector', 'numel', 4}, mfilename, 'Fx_wheel', 5);
    validateattributes(Fy_wheel, {'numeric'}, {'real', 'vector', 'numel', 4}, mfilename, 'Fy_wheel', 6);

    % Handle steering vector
    if isscalar(delta)
        delta_vec = [delta; delta; 0.0; 0.0];
    elseif numel(delta) == 4
        delta_vec = delta(:);
    else
        error('fourWheelDynamics:invalidDelta', 'delta must be a scalar or a 4-element vector.');
    end

    Fx_w = Fx_wheel(:);
    Fy_w = Fy_wheel(:);

    % 1. Force transformation from wheel coordinate frames to vehicle body frame
    %    [Fx_body; Fy_body] = R(-delta) * [Fx_wheel; Fy_wheel]
    %    Fx_body =  cos(delta) * Fx_wheel - sin(delta) * Fy_wheel
    %    Fy_body =  sin(delta) * Fx_wheel + cos(delta) * Fy_wheel
    cos_d = cos(delta_vec);
    sin_d = sin(delta_vec);

    Fx_body =  cos_d .* Fx_w - sin_d .* Fy_w;
    Fy_body =  sin_d .* Fx_w + cos_d .* Fy_w;

    % 2. Total body forces
    Fx_total = sum(Fx_body);
    Fy_total = sum(Fy_body);

    % 3. Individual and total yaw moments about CG
    pos = params.wheelPositions; % 4x2 matrix [x_i, y_i]
    xi = pos(:, 1);
    yi = pos(:, 2);

    % Mz_i = (r_i x F_body_i)_z = x_i * Fy_body_i - y_i * Fx_body_i
    Mz_i = xi .* Fy_body - yi .* Fx_body;
    Mz_total = sum(Mz_i);

    % 4. 3-DOF Planar Equations of Motion
    %    dot(vx) = (Fx_total / m) + vy * r
    %    dot(vy) = (Fy_total / m) - vx * r
    %    dot(r)  = Mz_total / Iz
    dot_vx = (Fx_total / params.m) + (vy * r);
    dot_vy = (Fy_total / params.m) - (vx * r);
    dot_r  = Mz_total / params.Iz;

    state_deriv = [dot_vx; dot_vy; dot_r];

    if nargout > 1
        dynamics.Fx_body   = Fx_body;
        dynamics.Fy_body   = Fy_body;
        dynamics.Fx_total  = Fx_total;
        dynamics.Fy_total  = Fy_total;
        dynamics.Mz_i      = Mz_i;
        dynamics.Mz_total  = Mz_total;
        dynamics.dot_vx    = dot_vx;
        dynamics.dot_vy    = dot_vy;
        dynamics.dot_r     = dot_r;
    end
end
