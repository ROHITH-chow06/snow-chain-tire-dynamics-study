function [Fz, Fz_axles] = normalLoadDistribution(params)
% NORMALLOADDISTRIBUTION Calculate static four-wheel normal vertical loads.
%
%   Fz = normalLoadDistribution()
%   Fz = normalLoadDistribution(params)
%   [Fz, Fz_axles] = normalLoadDistribution(params)
%
%   Computes the static normal load distribution for the planar four-wheel
%   rover under gravitational equilibrium.
%
%   Assumptions:
%       - Static weight distribution without dynamic acceleration load transfer.
%       - Symmetric lateral CG placement (equal left/right distribution per axle).
%       - SI units: mass in kg, dimensions in m, gravity in m/s^2, forces in N.
%
%   Inputs:
%       params   - (Optional) Rover parameter struct from roverParameters()
%
%   Outputs:
%       Fz       - 4x1 vector of vertical normal loads [N]
%                  Index 1: FL (Front-Left)
%                  Index 2: FR (Front-Right)
%                  Index 3: RL (Rear-Left)
%                  Index 4: RR (Rear-Right)
%       Fz_axles - Struct containing total and per-axle loads [N]:
%                  .total - Total vertical vehicle load (m * g)
%                  .front - Front axle load (m * g * lr / (lf + lr))
%                  .rear  - Rear axle load (m * g * lf / (lf + lr))

    if nargin < 1 || isempty(params)
        params = roverParameters();
    end

    % Total vertical load
    Fz_total = params.m * params.g;

    % Axle loads based on longitudinal CG location
    L = params.lf + params.lr;
    Fz_front = Fz_total * (params.lr / L);
    Fz_rear  = Fz_total * (params.lf / L);

    % Equal lateral split per axle
    Fz_FL = Fz_front / 2.0;
    Fz_FR = Fz_front / 2.0;
    Fz_RL = Fz_rear  / 2.0;
    Fz_RR = Fz_rear  / 2.0;

    Fz = [Fz_FL; Fz_FR; Fz_RL; Fz_RR];

    if nargout > 1
        Fz_axles.total = Fz_total;
        Fz_axles.front = Fz_front;
        Fz_axles.rear  = Fz_rear;
    end
end
