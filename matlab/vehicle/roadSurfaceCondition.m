function condition = roadSurfaceCondition(surfaceType)
% ROADSURFACECONDITION Generate representative tire-road parameters for comparative simulation.
%
%   condition = roadSurfaceCondition(surfaceType)
%
%   This function serves as a factory for road-surface condition parameter sets.
%   It outputs structurally identical sets of 'pure_params' and 'combined_params'
%   designed for insertion into the Phase 3 Magic Formula tire equations.
%
%   Supported conditions (case-insensitive):
%       'Dry'  - High friction, stiff interface
%       'Wet'  - Reduced friction, slightly compliant interface
%       'Snow' - Substantially reduced friction, highly compliant shear interface
%
%   Output:
%       condition - Struct containing:
%           .type             - Normalized string name of the condition
%           .pure_params      - Struct of pure-slip MF coefficients (Bx, Cx, Dx, Ex, By, Cy, Dy, Ey)
%           .combined_params  - Struct of combined-slip weighting coefficients (Bx_alpha, etc.)
%
%   RESEARCH INTEGRITY STATEMENT:
%       These values are representative coefficient modifications selected to 
%       produce controlled differences in force capacity and slip sensitivity 
%       between surface-condition cases.
%       The D values are representative parameter choices used to create a 
%       simplified road-surface sensitivity framework. They are not universal 
%       experimentally measured friction coefficients for dry, wet, or 
%       snow-covered roads. Under the constant-coefficient Magic Formula 
%       used here, peak longitudinal force scales approximately with D*Fz.
%       These values are NOT experimentally measured commercial tire data and 
%       are intended exclusively for comparative mathematical characterization 
%       of a robotic ground vehicle.

    validateattributes(surfaceType, {'char', 'string'}, {'nonempty', 'scalartext'}, mfilename, 'surfaceType', 1);
    
    typeStr = char(lower(surfaceType));
    
    switch typeStr
        case 'dry'
            condition.type = 'Dry';
            
            % Pure-slip coefficients
            condition.pure_params = struct(...
                'Bx', 10.0, 'Cx', 1.90, 'Dx', 1.00, 'Ex', 0.97, ...
                'By', 10.0, 'Cy', 1.90, 'Dy', 1.00, 'Ey', 0.97);
            
            % Combined-slip coefficients
            condition.combined_params = struct(...
                'Bx_alpha', 10.0, 'Cx_alpha', 1.00, 'Ex_alpha', -1.00, 'SHx_alpha', 0.0, ...
                'By_kappa', 10.0, 'Cy_kappa', 1.00, 'Ey_kappa', -1.00, 'SHy_kappa', 0.0);
                
        case 'wet'
            condition.type = 'Wet';
            
            % Pure-slip coefficients
            condition.pure_params = struct(...
                'Bx', 8.0, 'Cx', 1.60, 'Dx', 0.70, 'Ex', 0.97, ...
                'By', 8.0, 'Cy', 1.60, 'Dy', 0.70, 'Ey', 0.97);
            
            % Combined-slip coefficients
            % Shape factor C reduced to widen the interaction envelope
            condition.combined_params = struct(...
                'Bx_alpha', 8.0, 'Cx_alpha', 0.85, 'Ex_alpha', -1.00, 'SHx_alpha', 0.0, ...
                'By_kappa', 8.0, 'Cy_kappa', 0.85, 'Ey_kappa', -1.00, 'SHy_kappa', 0.0);
                
        case 'snow'
            condition.type = 'Snow';
            
            % Pure-slip coefficients
            condition.pure_params = struct(...
                'Bx', 4.0, 'Cx', 1.20, 'Dx', 0.30, 'Ex', 0.97, ...
                'By', 4.0, 'Cy', 1.20, 'Dy', 0.30, 'Ey', 0.97);
            
            % Combined-slip coefficients
            % Shape factor C further reduced for highly compliant shear behavior
            condition.combined_params = struct(...
                'Bx_alpha', 4.0, 'Cx_alpha', 0.70, 'Ex_alpha', -1.00, 'SHx_alpha', 0.0, ...
                'By_kappa', 4.0, 'Cy_kappa', 0.70, 'Ey_kappa', -1.00, 'SHy_kappa', 0.0);
                
        otherwise
            error('roadSurfaceCondition:invalidType', ...
                'Condition type "%s" is not supported. Use ''Dry'', ''Wet'', or ''Snow''.', surfaceType);
    end
end
