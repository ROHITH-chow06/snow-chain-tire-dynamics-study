function [Fz, diagnostics] = dynamicNormalLoadDistribution(varargin)
% DYNAMICNORMALLOADDISTRIBUTION Calculate quasi-static four-wheel dynamic vertical loads.
%
%   Fz = dynamicNormalLoadDistribution(m, g, lf, lr, tf, tr, hCG, ax, ay)
%   Fz = dynamicNormalLoadDistribution(params, ax, ay)
%   [Fz, diagnostics] = dynamicNormalLoadDistribution(...)
%
%   Computes the quasi-static four-wheel normal load distribution for a planar
%   ground vehicle subjected to longitudinal and lateral accelerations.
%
%   Sign Conventions:
%       +ax : Forward vehicle acceleration (load transfers from front to rear)
%       +ay : Leftward vehicle acceleration (load transfers from left to right/outside)
%       +z  : Upward (normal loads Fz are positive upward contact forces)
%
%   Wheel Ordering:
%       Index 1: FL (Front-Left)
%       Index 2: FR (Front-Right)
%       Index 3: RL (Rear-Left)
%       Index 4: RR (Rear-Right)
%
%   Inputs:
%       Option A (9 arguments):
%           m   - Total vehicle mass [kg] (> 0)
%           g   - Gravitational acceleration [m/s^2] (> 0)
%           lf  - Distance from CG to front axle [m] (> 0)
%           lr  - Distance from CG to rear axle [m] (> 0)
%           tf  - Front axle track width [m] (> 0)
%           tr  - Rear axle track width [m] (> 0)
%           hCG - Center of gravity height above ground [m] (> 0)
%           ax  - Longitudinal acceleration [m/s^2]
%           ay  - Lateral acceleration [m/s^2]
%
%       Option B (3 arguments):
%           params - Struct containing fields: m, g, lf, lr, tf, tr, hCG
%           ax     - Longitudinal acceleration [m/s^2]
%           ay     - Lateral acceleration [m/s^2]
%
%   Outputs:
%       Fz          - 4x1 vector of dynamic vertical normal loads [N]
%       diagnostics - Struct containing intermediate quantities:
%                     .Fz_static         - 4x1 static normal loads [N]
%                     .DeltaFz_long      - Total axle longitudinal load transfer [N]
%                     .DeltaFz_lat_front - Front wheel lateral load transfer [N]
%                     .DeltaFz_lat_rear  - Rear wheel lateral load transfer [N]
%                     .lambda_f          - Front roll moment distribution fraction
%                     .lambda_r          - Rear roll moment distribution fraction
%
%   Validity:
%       Rejects cases where load transfer causes wheel lift (Fz <= 0).

    % Parse inputs
    if nargin == 3
        params = varargin{1};
        ax = varargin{2};
        ay = varargin{3};
        if ~isstruct(params)
            error('dynamicNormalLoadDistribution:invalidParams', 'First argument must be a parameter struct.');
        end
        reqFields = {'m', 'g', 'lf', 'lr', 'tf', 'tr', 'hCG'};
        for k = 1:length(reqFields)
            if ~isfield(params, reqFields{k})
                error('dynamicNormalLoadDistribution:missingField', 'Parameter struct missing field: %s', reqFields{k});
            end
        end
        m   = params.m;
        g   = params.g;
        lf  = params.lf;
        lr  = params.lr;
        tf  = params.tf;
        tr  = params.tr;
        hCG = params.hCG;
    elseif nargin == 9
        m   = varargin{1};
        g   = varargin{2};
        lf  = varargin{3};
        lr  = varargin{4};
        tf  = varargin{5};
        tr  = varargin{6};
        hCG = varargin{7};
        ax  = varargin{8};
        ay  = varargin{9};
    else
        error('dynamicNormalLoadDistribution:invalidNumArgs', ...
              'Function accepts either 3 arguments (params, ax, ay) or 9 arguments (m, g, lf, lr, tf, tr, hCG, ax, ay).');
    end

    % Validate parameters
    if ~isnumeric(m) || ~isscalar(m) || m <= 0 || ...
       ~isnumeric(g) || ~isscalar(g) || g <= 0 || ...
       ~isnumeric(lf) || ~isscalar(lf) || lf <= 0 || ...
       ~isnumeric(lr) || ~isscalar(lr) || lr <= 0 || ...
       ~isnumeric(tf) || ~isscalar(tf) || tf <= 0 || ...
       ~isnumeric(tr) || ~isscalar(tr) || tr <= 0 || ...
       ~isnumeric(hCG) || ~isscalar(hCG) || hCG <= 0
        error('dynamicNormalLoadDistribution:invalidParameters', ...
              'Vehicle parameters (m, g, lf, lr, tf, tr, hCG) must be positive real scalars.');
    end

    if ~isnumeric(ax) || ~isscalar(ax) || ~isnumeric(ay) || ~isscalar(ay)
        error('dynamicNormalLoadDistribution:invalidAcceleration', ...
              'Accelerations ax and ay must be real scalars.');
    end

    % Wheelbase and total static normal load
    L = lf + lr;
    Fz_total = m * g;

    % Static axle loads
    Fz_front_static = Fz_total * (lr / L);
    Fz_rear_static  = Fz_total * (lf / L);

    % Static individual wheel loads (symmetric left/right)
    Fz_FL_static = Fz_front_static / 2.0;
    Fz_FR_static = Fz_front_static / 2.0;
    Fz_RL_static = Fz_rear_static  / 2.0;
    Fz_RR_static = Fz_rear_static  / 2.0;

    Fz_static = [Fz_FL_static; Fz_FR_static; Fz_RL_static; Fz_RR_static];

    % Longitudinal load transfer (positive ax transfers load from front to rear)
    DeltaFz_long = (m * ax * hCG) / L;
    dFz_long_FL = -DeltaFz_long / 2.0;
    dFz_long_FR = -DeltaFz_long / 2.0;
    dFz_long_RL = +DeltaFz_long / 2.0;
    dFz_long_RR = +DeltaFz_long / 2.0;

    % Lateral load transfer distribution (based on static axle load fraction)
    lambda_f = lr / L;
    lambda_r = lf / L;

    % Lateral load shift per wheel on front and rear axles
    % Positive ay (leftward) shifts load to right/outside wheels (FR, RR gain load, FL, RL lose load)
    DeltaFz_lat_front = (lambda_f * m * ay * hCG) / tf;
    DeltaFz_lat_rear  = (lambda_r * m * ay * hCG) / tr;

    dFz_lat_FL = -DeltaFz_lat_front;
    dFz_lat_FR = +DeltaFz_lat_front;
    dFz_lat_RL = -DeltaFz_lat_rear;
    dFz_lat_RR = +DeltaFz_lat_rear;

    % Combined individual wheel normal loads
    Fz_FL = Fz_FL_static + dFz_long_FL + dFz_lat_FL;
    Fz_FR = Fz_FR_static + dFz_long_FR + dFz_lat_FR;
    Fz_RL = Fz_RL_static + dFz_long_RL + dFz_lat_RL;
    Fz_RR = Fz_RR_static + dFz_long_RR + dFz_lat_RR;

    Fz = [Fz_FL; Fz_FR; Fz_RL; Fz_RR];

    % Physical validity check: Wheel normal loads must be strictly positive
    if any(Fz <= 0)
        error('dynamicNormalLoadDistribution:negativeNormalLoad', ...
              'Calculated wheel normal load is non-positive (wheel lift detected: min(Fz) = %.3f N).', min(Fz));
    end

    % Diagnostic outputs
    if nargout > 1
        diagnostics.Fz_static         = Fz_static;
        diagnostics.Fz_front_static   = Fz_front_static;
        diagnostics.Fz_rear_static    = Fz_rear_static;
        diagnostics.DeltaFz_long      = DeltaFz_long;
        diagnostics.DeltaFz_lat_front = DeltaFz_lat_front;
        diagnostics.DeltaFz_lat_rear  = DeltaFz_lat_rear;
        diagnostics.lambda_f          = lambda_f;
        diagnostics.lambda_r          = lambda_r;
    end
end
