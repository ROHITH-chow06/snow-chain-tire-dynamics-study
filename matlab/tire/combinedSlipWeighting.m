function G = combinedSlipWeighting(x, B, C, E, SH)
% COMBINEDSLIPWEIGHTING Evaluates the PAC2002 combined-slip weighting function.
%
%   G = combinedSlipWeighting(x, B, C, E, SH) computes the weighting factor
%   used to reduce pure-slip tire forces under combined slip conditions.
%
%   Inputs:
%       x  - Primary slip variable causing force reduction 
%            (alpha for longitudinal force, kappa for lateral force).
%       B  - Stiffness factor.
%       C  - Shape factor.
%       E  - Curvature factor.
%       SH - Horizontal shift.
%
%   Output:
%       G  - Normalized weighting factor (G(0) = 1).
%
%   The function uses the PAC2002 cosine formulation:
%       G(x) = cos(C * atan(B*xs - E*(B*xs - atan(B*xs)))) / G0
%   where xs = x + SH, and G0 is the evaluation at x = 0.

    % Input validation
    if nargin < 5
        error('combinedSlipWeighting:notEnoughInputs', 'Requires 5 inputs: x, B, C, E, SH.');
    end
    if ~isreal(x) || ~isnumeric(x)
        error('combinedSlipWeighting:invalidInput', 'Input x must be a real numeric array.');
    end
    if ~isscalar(B) || ~isscalar(C) || ~isscalar(E) || ~isscalar(SH)
        error('combinedSlipWeighting:invalidParameters', 'Parameters B, C, E, SH must be scalars.');
    end

    % Evaluate denominator (G0) at x = 0 for normalization
    B_SH = B * SH;
    inner_0 = B_SH - E * (B_SH - atan(B_SH));
    G0 = cos(C * atan(inner_0));
    
    if abs(G0) < 1e-12
        error('combinedSlipWeighting:zeroNormalization', 'Denominator G0 is zero for the given parameters.');
    end

    % Evaluate weighting function at shifted input
    xs = x + SH;
    B_xs = B .* xs;
    inner_xs = B_xs - E .* (B_xs - atan(B_xs));
    G_unnorm = cos(C .* atan(inner_xs));

    % Normalized weighting factor
    G = G_unnorm ./ G0;
end
