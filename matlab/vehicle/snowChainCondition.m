function condition = snowChainCondition()
% SNOWCHAINCONDITION Generate representative tire-road parameters for a simplified comparative benchmark.
%
%   condition = snowChainCondition()
%
%   This function serves as a factory for a "SnowChain" parameter set,
%   designed as a simplified comparative benchmark against a baseline normal tire on snow.
%
%   Output:
%       condition - Struct containing:
%           .type             - 'SnowChain'
%           .pure_params      - Struct of pure-slip MF coefficients (Bx, Cx, Dx, Ex, By, Cy, Dy, Ey)
%           .combined_params  - Struct of combined-slip weighting coefficients (Bx_alpha, etc.)
%
%   RESEARCH INTEGRITY STATEMENT:
%       SnowChain is a simplified numerical sensitivity benchmark, not a 
%       calibrated physical model of a commercial snow chain.
%       The 1.5x longitudinal Dx modification is an illustrative modeling 
%       assumption used for sensitivity analysis, not an experimentally 
%       measured universal chain multiplier.
%       This model does not represent discrete chain links, contact geometry, 
%       digging, chain slip, impact/chatter, or detailed tire-chain-snow mechanics.
%       The results demonstrate mathematical sensitivity, and should not be 
%       interpreted as experimental validation or predicting the performance 
%       of a physical commercial chain.

    % 1. Get the baseline unchained snow condition
    base = roadSurfaceCondition('Snow');
    
    % 2. Define explicit macroscopic modeling assumptions
    % An illustrative longitudinal enhancement assumption (sensitivity-study parameter)
    % representing macroscopic tractive benefits.
    longitudinal_enhancement = 1.5;
    
    % 3. Construct the new condition struct
    condition.type = 'SnowChain';
    
    % Copy all pure-slip parameters identically
    condition.pure_params = base.pure_params;
    % Explicitly scale ONLY the longitudinal peak friction limit
    condition.pure_params.Dx = condition.pure_params.Dx * longitudinal_enhancement;
    
    % Copy all combined-slip parameters identically
    condition.combined_params = base.combined_params;
end
