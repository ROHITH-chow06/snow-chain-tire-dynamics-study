function kinematics = wheelKinematics(vx, vy, r, delta, omega, params, epsilon_slip, v_threshold)
% WHEELKINEMATICS Compute planar four-wheel rover kinematics.
%
%   kinematics = wheelKinematics(vx, vy, r, delta, omega)
%   kinematics = wheelKinematics(vx, vy, r, delta, omega, params)
%   kinematics = wheelKinematics(vx, vy, r, delta, omega, params, epsilon_slip, v_threshold)
%
%   Computes local rigid-body velocities, wheel-frame velocities, longitudinal
%   slip ratios, and wheel slip angles for all four individual wheel locations:
%       Index 1: FL (Front-Left)
%       Index 2: FR (Front-Right)
%       Index 3: RL (Rear-Left)
%       Index 4: RR (Rear-Right)
%
%   Inputs:
%       vx           - Vehicle CG longitudinal body velocity [m/s] (scalar)
%       vy           - Vehicle CG lateral body velocity [m/s] (scalar)
%       r            - Vehicle yaw rate [rad/s] (scalar, positive CCW)
%       delta        - Front steering angle [rad] (scalar, or 4x1 vector [delta_FL; delta_FR; delta_RL; delta_RR])
%       omega        - Wheel rotational angular velocities [rad/s] (4x1 or 1x4 vector)
%       params       - (Optional) Rover parameter struct from roverParameters()
%       epsilon_slip - (Optional) Slip ratio regularization threshold [m/s] (default: 0.1)
%       v_threshold  - (Optional) Slip angle zero-speed threshold [m/s] (default: 1e-4)
%
%   Output fields:
%       vx_body      - 4x1 vector of wheel velocities along vehicle body x-axis [m/s]
%       vy_body      - 4x1 vector of wheel velocities along vehicle body y-axis [m/s]
%       vx_wheel     - 4x1 vector of wheel velocities along wheel x-axis [m/s]
%       vy_wheel     - 4x1 vector of wheel velocities along wheel y-axis [m/s]
%       kappa        - 4x1 vector of longitudinal slip ratios [-]
%       alpha        - 4x1 vector of signed slip angles [rad]
%       delta        - 4x1 vector of wheel steering angles [rad]
%       wheelNames   - 1x4 cell array: {'FL', 'FR', 'RL', 'RR'}

    % Default parameters
    if nargin < 6 || isempty(params)
        params = roverParameters();
    end
    if nargin < 7 || isempty(epsilon_slip)
        epsilon_slip = 0.1;
    end
    if nargin < 8 || isempty(v_threshold)
        v_threshold = 1e-4;
    end

    % Input validation
    validateattributes(vx, {'numeric'}, {'real', 'scalar'}, mfilename, 'vx', 1);
    validateattributes(vy, {'numeric'}, {'real', 'scalar'}, mfilename, 'vy', 2);
    validateattributes(r,  {'numeric'}, {'real', 'scalar'}, mfilename, 'r', 3);
    validateattributes(delta, {'numeric'}, {'real', 'vector'}, mfilename, 'delta', 4);
    validateattributes(omega, {'numeric'}, {'real', 'vector', 'numel', 4}, mfilename, 'omega', 5);

    % Handle steering vector (front-steered default if scalar)
    if isscalar(delta)
        delta_vec = [delta; delta; 0.0; 0.0];
    elseif numel(delta) == 4
        delta_vec = delta(:);
    else
        error('wheelKinematics:invalidDelta', 'delta must be a scalar (front axle) or a 4-element vector.');
    end

    omega_vec = omega(:);
    pos = params.wheelPositions; % 4x2 matrix [x_i, y_i]
    xi = pos(:, 1);
    yi = pos(:, 2);

    % 1. Rigid-body velocities in vehicle body frame: v_body,i = v_CG + omega x r_i
    vx_body = vx - r .* yi;
    vy_body = vy + r .* xi;

    % 2. 2D rotation into wheel coordinate frame: [vx_wheel; vy_wheel] = R(delta) * [vx_body; vy_body]
    cos_d = cos(delta_vec);
    sin_d = sin(delta_vec);

    vx_wheel =  cos_d .* vx_body + sin_d .* vy_body;
    vy_wheel = -sin_d .* vx_body + cos_d .* vy_body;

    % 3. Longitudinal slip ratio (calling validated Phase 1A function per wheel)
    kappa = zeros(4, 1);
    for i = 1:4
        kappa(i) = longitudinalSlipRatio(params.R, omega_vec(i), vx_wheel(i), epsilon_slip);
    end

    % 4. Wheel velocity / slip angle (calling validated wheelSlipAngle function per wheel)
    alpha = wheelSlipAngle(vy_wheel, vx_wheel, v_threshold);

    % Populate output structure
    kinematics.vx_body    = vx_body;
    kinematics.vy_body    = vy_body;
    kinematics.vx_wheel   = vx_wheel;
    kinematics.vy_wheel   = vy_wheel;
    kinematics.kappa      = kappa;
    kinematics.alpha      = alpha;
    kinematics.delta      = delta_vec;
    kinematics.wheelNames = params.wheelNames;
end
