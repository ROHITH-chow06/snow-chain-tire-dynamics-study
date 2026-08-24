function params = roverParameters()
% ROVERPARAMETERS Return representative geometric, mass, and inertial parameters.
%
%   params = roverParameters()
%
%   Outputs a struct containing representative parameters for a four-wheel
%   research ground rover in SI units.
%
%   Output fields:
%       m              - Vehicle total mass [kg]
%       lf             - Distance from CG to front axle [m]
%       lr             - Distance from CG to rear axle [m]
%       tf             - Front track width [m]
%       tr             - Rear track width [m]
%       R              - Effective wheel rolling radius [m]
%       Iz             - Vehicle yaw moment of inertia about z-axis [kg*m^2]
%       g              - Gravitational acceleration [m/s^2]
%       L              - Total wheelbase (lf + lr) [m]
%       W              - Representative vehicle width [m]
%       wheelPositions - 4x2 matrix of wheel coordinates [x, y] relative to CG [m]
%                        Row 1: FL (+lf, +tf/2)
%                        Row 2: FR (+lf, -tf/2)
%                        Row 3: RL (-lr, +tr/2)
%                        Row 4: RR (-lr, -tr/2)
%       wheelNames     - 1x4 cell array: {'FL', 'FR', 'RL', 'RR'}
%
%   Coordinate Frame & Sign Convention:
%       Vehicle body-fixed frame:
%           +x : Forward
%           +y : Left
%           +z : Upward
%       Yaw rate r is positive counter-clockwise (CCW) viewed from above (+z).
%
%   Inertia Derivation:
%       Iz is computed using a simplified planar rectangular mass distribution:
%           Iz = (m / 12) * (L^2 + W^2)
%       where L = lf + lr and W = tf. This is a transparent engineering
%       approximation and does not represent measured values of a commercial rover.

    % Vehicle mass and gravity
    params.m = 60.0;            % Total vehicle mass [kg]
    params.g = 9.81;            % Gravitational acceleration [m/s^2]

    % Dimensions and geometry
    params.lf  = 0.40;          % CG to front axle distance [m]
    params.lr  = 0.40;          % CG to rear axle distance [m]
    params.tf  = 0.60;          % Front axle track width [m]
    params.tr  = 0.60;          % Rear axle track width [m]
    params.R   = 0.15;          % Effective wheel rolling radius [m]
    params.hCG = 0.30;          % Center of gravity height above ground [m]
                                % Note: Representative modeling parameter for quasi-static
                                % load transfer, not an experimental measurement.

    params.L = params.lf + params.lr; % Wheelbase [m] (0.80 m)
    params.W = params.tf;             % Representative body width [m] (0.60 m)

    % Yaw moment of inertia (rectangular prism approximation)
    params.Iz = (params.m / 12.0) * (params.L^2 + params.W^2); % 5.00 kg*m^2

    % Wheel positions relative to CG [x, y] in vehicle body frame
    % Ordering: [FL; FR; RL; RR]
    params.wheelPositions = [
        +params.lf, +params.tf / 2.0;   % FL: Front-Left
        +params.lf, -params.tf / 2.0;   % FR: Front-Right
        -params.lr, +params.tr / 2.0;   % RL: Rear-Left
        -params.lr, -params.tr / 2.0    % RR: Rear-Right
    ];

    params.wheelNames = {'FL', 'FR', 'RL', 'RR'};
end
