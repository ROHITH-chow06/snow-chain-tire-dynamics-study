% compareRoadConditions.m
% Controlled comparative simulation of longitudinal tire force under
% Dry, Wet, and Snow road conditions using the constant-coefficient Magic Formula.
%
% Generates:
%   1. Primary force-slip comparison plot -> results/figures/road_condition_longitudinal_tire_force.png
%   2. Peak force summary bar chart      -> results/figures/road_condition_peak_force_comparison.png
%   3. Full sweep dataset               -> results/data/road_condition_longitudinal_tire_force.csv
%   4. Comparative metrics summary table -> results/data/road_condition_summary.csv

clear;
clc;

% Resolve workspace root and function paths
rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(rootDir, 'matlab', 'tire'));

% Establish output directories
figDir = fullfile(rootDir, 'results', 'figures');
dataDir = fullfile(rootDir, 'results', 'data');
if ~exist(figDir, 'dir'), mkdir(figDir); end
if ~exist(dataDir, 'dir'), mkdir(dataDir); end

% 1. Controlled Experimental Variables
Fz = 4000.0;    % Vertical tire normal load [N] (held constant across conditions)
nPoints = 2001; % Discretization resolution
kappa = linspace(-1.0, 1.0, nPoints)';

% 2. Sourced Representative Parameter Sets (Canonical Phase 3J Conditions)
% These are fetched directly from the unified roadSurfaceCondition factory.
c_dry = roadSurfaceCondition('Dry');
dry.B = c_dry.pure_params.Bx; dry.C = c_dry.pure_params.Cx; dry.D = c_dry.pure_params.Dx; dry.E = c_dry.pure_params.Ex;

c_wet = roadSurfaceCondition('Wet');
wet.B = c_wet.pure_params.Bx; wet.C = c_wet.pure_params.Cx; wet.D = c_wet.pure_params.Dx; wet.E = c_wet.pure_params.Ex;

c_snow = roadSurfaceCondition('Snow');
snow.B = c_snow.pure_params.Bx; snow.C = c_snow.pure_params.Cx; snow.D = c_snow.pure_params.Dx; snow.E = c_snow.pure_params.Ex;

% 3. Force Evaluation using the validated Phase 1B function
Fx_dry  = magicFormulaLongitudinal(kappa, Fz, dry.B,  dry.C,  dry.D,  dry.E);
Fx_wet  = magicFormulaLongitudinal(kappa, Fz, wet.B,  wet.C,  wet.D,  wet.E);
Fx_snow = magicFormulaLongitudinal(kappa, Fz, snow.B, snow.C, snow.D, snow.E);

% 4. Quantitative Metrics Calculation
% Initial tangent stiffness K0 = B * C * D * Fz
K0_dry  = dry.B  * dry.C  * dry.D  * Fz;
K0_wet  = wet.B  * wet.C  * wet.D  * Fz;
K0_snow = snow.B * snow.C * snow.D * Fz;

% Peak forces and slip locations (positive driving branch)
posIdx = (kappa >= 0);
kappa_pos = kappa(posIdx);

[Fx_dry_peak,  idx_dry]  = max(Fx_dry(posIdx));   kappa_dry_peak  = kappa_pos(idx_dry);
[Fx_wet_peak,  idx_wet]  = max(Fx_wet(posIdx));   kappa_wet_peak  = kappa_pos(idx_wet);
[Fx_snow_peak, idx_snow] = max(Fx_snow(posIdx));  kappa_snow_peak = kappa_pos(idx_snow);

% Peak braking forces (negative braking branch)
negIdx = (kappa <= 0);
kappa_neg = kappa(negIdx);

[Fx_dry_brake_mag,  idx_dry_b]  = max(abs(Fx_dry(negIdx)));   kappa_dry_brake  = kappa_neg(idx_dry_b);
[Fx_wet_brake_mag,  idx_wet_b]  = max(abs(Fx_wet(negIdx)));   kappa_wet_brake  = kappa_neg(idx_wet_b);
[Fx_snow_brake_mag, idx_snow_b] = max(abs(Fx_snow(negIdx)));  kappa_snow_brake = kappa_neg(idx_snow_b);

% Achieved force-to-normal-load ratios over the evaluated slip range
fx_over_fz_dry  = Fx_dry_peak  / Fz;
fx_over_fz_wet  = Fx_wet_peak  / Fz;
fx_over_fz_snow = Fx_snow_peak / Fz;

% Forces at selected operating slip ratios: kappa = 0.05, 0.10, 0.20
target_kappas = [0.05; 0.10; 0.20];
Fx_dry_sel  = magicFormulaLongitudinal(target_kappas, Fz, dry.B,  dry.C,  dry.D,  dry.E);
Fx_wet_sel  = magicFormulaLongitudinal(target_kappas, Fz, wet.B,  wet.C,  wet.D,  wet.E);
Fx_snow_sel = magicFormulaLongitudinal(target_kappas, Fz, snow.B, snow.C, snow.D, snow.E);

% Relative percentage changes compared to Dry baseline
pct_peak_wet  = ((Fx_wet_peak  - Fx_dry_peak)  / Fx_dry_peak)  * 100;
pct_peak_snow = ((Fx_snow_peak - Fx_dry_peak)  / Fx_dry_peak)  * 100;

pct_K0_wet    = ((K0_wet  - K0_dry)  / K0_dry)  * 100;
pct_K0_snow   = ((K0_snow - K0_dry)  / K0_dry)  * 100;

pct_sel_wet   = ((Fx_wet_sel  - Fx_dry_sel)  ./ Fx_dry_sel)  .* 100;
pct_sel_snow  = ((Fx_snow_sel - Fx_dry_sel) ./ Fx_dry_sel) .* 100;

% 5. Verification & Physical Sanity Checks
tol = 1e-12;
zeroIdx = (kappa == 0);
assert(abs(Fx_dry(zeroIdx)) < tol && abs(Fx_wet(zeroIdx)) < tol && abs(Fx_snow(zeroIdx)) < tol, ...
    'Check Failed: Zero slip did not produce zero force across all conditions.');
assert(all(isfinite(Fx_dry)) && all(isfinite(Fx_wet)) && all(isfinite(Fx_snow)), ...
    'Check Failed: Non-finite values detected.');
assert(~isequal(Fx_dry, Fx_wet) && ~isequal(Fx_dry, Fx_snow), ...
    'Check Failed: Road condition responses are unexpectedly identical.');

% Baseline reproduction check against Phase 1C committed baseline
assert(abs(Fx_dry_peak - 4000.0) < 0.01, 'Check Failed: Dry peak force deviates from Phase 1C baseline.');
assert(abs(kappa_dry_peak - 0.1800) < 1e-4, 'Check Failed: Dry peak slip deviates from Phase 1C baseline.');
assert(abs(K0_dry - 76000.0) < tol, 'Check Failed: Dry initial stiffness deviates from Phase 1C baseline.');

% 6. Display Formatted Console Summary
fprintf('========================================================================================\n');
fprintf('  Phase 3J: Road Condition Parameterization & Comparative Modeling Summary\n');
fprintf('========================================================================================\n');
fprintf('Normal Load (Fz): %.1f N (Held constant for all conditions)\n\n', Fz);

fprintf('%-12s | %-6s %-6s %-6s %-6s | %-12s | %-10s | %-8s | %-12s | %-14s\n', ...
    'Condition', 'B', 'C', 'D', 'E', 'Max Fx [N]', 'Max kappa', 'D_param', 'Max Fx / Fz', 'K0 [N/slip]');
fprintf('------------------------------------------------------------------------------------------------------\n');
fprintf('%-12s | %-6.1f %-6.2f %-6.2f %-6.2f | %-12.2f | %-10.4f | %-8.2f | %-12.4f | %-14.1f\n', ...
    'Dry Tarmac', dry.B, dry.C, dry.D, dry.E, Fx_dry_peak, kappa_dry_peak, dry.D, fx_over_fz_dry, K0_dry);
fprintf('%-12s | %-6.1f %-6.2f %-6.2f %-6.2f | %-12.2f | %-10.4f | %-8.2f | %-12.4f | %-14.1f\n', ...
    'Wet Tarmac', wet.B, wet.C, wet.D, wet.E, Fx_wet_peak, kappa_wet_peak, wet.D, fx_over_fz_wet, K0_wet);
fprintf('%-12s | %-6.1f %-6.2f %-6.2f %-6.2f | %-12.2f | %-10.4f | %-8.2f | %-12.4f | %-14.1f\n', ...
    'Snow', snow.B, snow.C, snow.D, snow.E, Fx_snow_peak, kappa_snow_peak, snow.D, fx_over_fz_snow, K0_snow);
fprintf('----------------------------------------------------------------------------------------\n\n');

fprintf('Force Comparison at Selected Slip Ratios:\n');
fprintf('%-12s | %-16s | %-16s | %-16s\n', 'Condition', 'Fx(kappa=0.05)', 'Fx(kappa=0.10)', 'Fx(kappa=0.20)');
fprintf('----------------------------------------------------------------------------------------\n');
fprintf('%-12s | %-16.2f | %-16.2f | %-16.2f\n', 'Dry Tarmac', Fx_dry_sel(1), Fx_dry_sel(2), Fx_dry_sel(3));
fprintf('%-12s | %-16.2f | %-16.2f | %-16.2f\n', 'Wet Tarmac', Fx_wet_sel(1), Fx_wet_sel(2), Fx_wet_sel(3));
fprintf('%-12s | %-16.2f | %-16.2f | %-16.2f\n', 'Snow', Fx_snow_sel(1), Fx_snow_sel(2), Fx_snow_sel(3));
fprintf('----------------------------------------------------------------------------------------\n\n');

fprintf('Relative Change Compared to Dry Baseline:\n');
fprintf('  * Wet Tarmac : Peak Force = %+.2f%% | Initial Stiffness K0 = %+.2f%%\n', pct_peak_wet, pct_K0_wet);
fprintf('                 Fx(0.05)   = %+.2f%% | Fx(0.10) = %+.2f%% | Fx(0.20) = %+.2f%%\n', ...
    pct_sel_wet(1), pct_sel_wet(2), pct_sel_wet(3));
fprintf('  * Snow Road  : Peak Force = %+.2f%% | Initial Stiffness K0 = %+.2f%%\n', pct_peak_snow, pct_K0_snow);
fprintf('                 Fx(0.05)   = %+.2f%% | Fx(0.10) = %+.2f%% | Fx(0.20) = %+.2f%%\n', ...
    pct_sel_snow(1), pct_sel_snow(2), pct_sel_snow(3));
fprintf('========================================================================================\n\n');

% 7. Primary Visualization: Road Condition Longitudinal Tire Force Curves
hFig1 = figure('Name', 'Road Condition Force Comparison', 'Color', 'w', 'Position', [100, 100, 850, 520], 'Visible', 'off');
plot(kappa, Fx_dry,  'b-',  'LineWidth', 2.0, 'DisplayName', sprintf('Dry Tarmac (D = %.2f)', dry.D));
hold on;
plot(kappa, Fx_wet,  'r--', 'LineWidth', 2.0, 'DisplayName', sprintf('Wet Tarmac (D = %.2f)', wet.D));
plot(kappa, Fx_snow, 'm-.', 'LineWidth', 2.0, 'DisplayName', sprintf('Snow (D = %.2f)', snow.D));

% Highlight peak points
plot(kappa_dry_peak,  Fx_dry_peak,  'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'HandleVisibility', 'off');
plot(kappa_wet_peak,  Fx_wet_peak,  'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6, 'HandleVisibility', 'off');
plot(kappa_snow_peak, Fx_snow_peak, 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 6, 'HandleVisibility', 'off');

% Highlight braking peak points
plot(kappa_dry_brake,  -Fx_dry_brake_mag,  'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'HandleVisibility', 'off');
plot(kappa_wet_brake,  -Fx_wet_brake_mag,  'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6, 'HandleVisibility', 'off');
plot(kappa_snow_brake, -Fx_snow_brake_mag, 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 6, 'HandleVisibility', 'off');

% Reference axes
plot([0, 0], [-4500, 4500], 'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
plot([-1.05, 1.05], [0, 0], 'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
hold off;

grid on;
box on;
xlabel('Longitudinal Slip Ratio \kappa [-]', 'FontSize', 11);
ylabel('Longitudinal Tire Force F_x [N]', 'FontSize', 11);
title('Comparative Longitudinal Tire Force: Dry vs. Wet vs. Snow (F_z = 4000 N)', 'FontSize', 12, 'FontWeight', 'bold');
xlim([-1.05, 1.05]);
ylim([-4500, 4500]);
legend('Location', 'northwest', 'FontSize', 10);

figFile1 = fullfile(figDir, 'road_condition_longitudinal_tire_force.png');
saveas(hFig1, figFile1);
close(hFig1);
fprintf('Saved primary comparison figure to: %s\n', figFile1);

% 8. Additional Figure: Peak Force and Initial Stiffness Comparison
hFig2 = figure('Name', 'Peak Force Comparison', 'Color', 'w', 'Position', [150, 150, 750, 450], 'Visible', 'off');

conditions = categorical({'Dry Tarmac', 'Wet Tarmac', 'Snow'});
conditions = reordercats(conditions, {'Dry Tarmac', 'Wet Tarmac', 'Snow'});
peak_forces = [Fx_dry_peak; Fx_wet_peak; Fx_snow_peak];

bar(conditions, peak_forces, 0.5, 'FaceColor', [0.2, 0.4, 0.8]);
grid on;
box on;
ylabel('Max Evaluated Longitudinal Force F_x [N]', 'FontSize', 11);
title('Maximum Evaluated Longitudinal Force by Road Surface Condition (F_z = 4000 N)', 'FontSize', 12, 'FontWeight', 'bold');
ylim([0, 5200]);

% Add text values above bars
actual_Ds = [dry.D, wet.D, snow.D];
for i = 1:length(peak_forces)
    text(i, peak_forces(i) + 250, sprintf('%.1f N\n(D = %.2f)\n[Max Fx/Fz = %.2f]', peak_forces(i), actual_Ds(i), peak_forces(i)/Fz), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

figFile2 = fullfile(figDir, 'road_condition_peak_force_comparison.png');
saveas(hFig2, figFile2);
close(hFig2);
fprintf('Saved peak force bar chart to: %s\n', figFile2);

% 9. Export Numerical Results
% Sweep data
sweepTable = table(kappa, Fx_dry, Fx_wet, Fx_snow, ...
    'VariableNames', {'kappa', 'Fx_dry_N', 'Fx_wet_N', 'Fx_snow_N'});
csvFileSweep = fullfile(dataDir, 'road_condition_longitudinal_tire_force.csv');
writetable(sweepTable, csvFileSweep);
fprintf('Saved full sweep dataset to: %s\n', csvFileSweep);

% Summary metrics table
condNames = {'Dry Tarmac'; 'Wet Tarmac'; 'Snow'};
B_vals    = [dry.B; wet.B; snow.B];
C_vals    = [dry.C; wet.C; snow.C];
D_vals    = [dry.D; wet.D; snow.D];
E_vals    = [dry.E; wet.E; snow.E];
Peak_Fx   = [Fx_dry_peak; Fx_wet_peak; Fx_snow_peak];
Peak_k    = [kappa_dry_peak; kappa_wet_peak; kappa_snow_peak];
Peak_Fx_over_Fz = [fx_over_fz_dry; fx_over_fz_wet; fx_over_fz_snow];
K0_vals   = [K0_dry; K0_wet; K0_snow];
Fx_k05    = [Fx_dry_sel(1); Fx_wet_sel(1); Fx_snow_sel(1)];
Fx_k10    = [Fx_dry_sel(2); Fx_wet_sel(2); Fx_snow_sel(2)];
Fx_k20    = [Fx_dry_sel(3); Fx_wet_sel(3); Fx_snow_sel(3)];
Delta_Fx_pct = [0.0; pct_peak_wet; pct_peak_snow];
Delta_K0_pct = [0.0; pct_K0_wet; pct_K0_snow];

summaryTable = table(condNames, B_vals, C_vals, D_vals, E_vals, ...
    Peak_Fx, Peak_k, D_vals, Peak_Fx_over_Fz, K0_vals, Fx_k05, Fx_k10, Fx_k20, Delta_Fx_pct, Delta_K0_pct, ...
    'VariableNames', {'Condition', 'B', 'C', 'D', 'E', ...
    'Max_Fx_N', 'Max_kappa', 'D_param', 'Peak_Fx_over_Fz', 'K0_N_per_slip', ...
    'Fx_kappa_005_N', 'Fx_kappa_010_N', 'Fx_kappa_020_N', ...
    'Max_Fx_RelChange_pct', 'K0_RelChange_pct'});

csvFileSummary = fullfile(dataDir, 'road_condition_summary.csv');
writetable(summaryTable, csvFileSummary);
fprintf('Saved summary metrics table to: %s\n\n', csvFileSummary);
