% characterizeBaselineTireForce.m
% Simulation and numerical characterization of the baseline constant-coefficient
% longitudinal tire force model.
%
% Generates:
%   1. Numerical metrics (peak force, peak slip, initial stiffness, mu_peak)
%   2. Plot saved to results/figures/baseline_longitudinal_tire_force.png
%   3. Sweep data saved to results/data/baseline_longitudinal_tire_force.csv

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

% Baseline representative model parameters (illustrative, not experimental data)
Fz = 4000.0;    % Vertical tire normal load [N]
B = 10.0;       % Stiffness factor [-]
C = 1.9;        % Shape factor [-]
D = 1.0;        % Peak factor [-]
E = 0.97;       % Curvature factor [-]

% 1. Slip-Ratio Sweep
nPoints = 2001;
kappa = linspace(-1.0, 1.0, nPoints)';

% 2. Force Evaluation using validated model
Fx = magicFormulaLongitudinal(kappa, Fz, B, C, D, E);

% 3. Numerical Characterization
% Analytical initial stiffness at zero slip
K0 = B * C * D * Fz;

% Positive slip (driving) characterization
posIdx = (kappa >= 0);
kappa_pos = kappa(posIdx);
Fx_pos = Fx(posIdx);
[Fx_pos_peak, maxPosIdx] = max(Fx_pos);
kappa_pos_peak = kappa_pos(maxPosIdx);

% Negative slip (braking) characterization
negIdx = (kappa <= 0);
kappa_neg = kappa(negIdx);
Fx_neg = Fx(negIdx);
[Fx_neg_peak_mag, maxNegIdx] = max(abs(Fx_neg));
Fx_neg_peak = Fx_neg(maxNegIdx);
kappa_neg_peak = kappa_neg(maxNegIdx);

% Effective peak friction coefficient
mu_peak = Fx_pos_peak / Fz;

% 4. Physical Checks & Assertions
tol = 1e-12;
zeroIdx = (kappa == 0);
assert(abs(Fx(zeroIdx)) < tol, 'Physical check failed: Fx at zero slip is non-zero.');
assert(all(Fx(kappa > 0) > 0), 'Physical check failed: Positive slip produced non-positive force.');
assert(all(Fx(kappa < 0) < 0), 'Physical check failed: Negative slip produced non-negative force.');
assert(abs(Fx_pos_peak - Fx_neg_peak_mag) < tol, 'Physical check failed: Peak force magnitude is asymmetric.');
assert(abs(kappa_pos_peak - abs(kappa_neg_peak)) < 1e-5, 'Physical check failed: Peak slip location is asymmetric.');

% 5. Print Characterization Summary
fprintf('============================================================\n');
fprintf('  Baseline Longitudinal Tire Force Characterization\n');
fprintf('============================================================\n');
fprintf('Normal Load (Fz)               : %.1f N\n', Fz);
fprintf('Magic Formula Parameters       : B=%.1f, C=%.2f, D=%.2f, E=%.2f\n', B, C, D, E);
fprintf('Sweep Range                    : kappa in [%.1f, %.1f] (%d points)\n', min(kappa), max(kappa), nPoints);
fprintf('------------------------------------------------------------\n');
fprintf('Analytical Initial Stiffness K0: %.1f N/unit slip\n', K0);
fprintf('Peak Driving Force (Fx_max)    : +%.2f N at kappa = +%.4f\n', Fx_pos_peak, kappa_pos_peak);
fprintf('Peak Braking Force (Fx_min)    : -%.2f N at kappa = %.4f\n', Fx_neg_peak_mag, kappa_neg_peak);
fprintf('Effective Peak Friction (mu)   : %.4f\n', mu_peak);
fprintf('Terminal Force at kappa = +1.0 : +%.2f N\n', Fx(end));
fprintf('Terminal Force at kappa = -1.0 : %.2f N\n', Fx(1));
fprintf('============================================================\n\n');

% 6. Visualization
hFig = figure('Name', 'Baseline Longitudinal Tire Force', 'Color', 'w', 'Position', [100, 100, 800, 500], 'Visible', 'off');
plot(kappa, Fx, 'b-', 'LineWidth', 1.8);
hold on;
plot(kappa_pos_peak, Fx_pos_peak, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
plot(kappa_neg_peak, Fx_neg_peak, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
plot([0, 0], [-4500, 4500], 'k--', 'LineWidth', 0.8);
plot([-1, 1], [0, 0], 'k--', 'LineWidth', 0.8);
hold off;

grid on;
box on;
xlabel('Longitudinal Slip Ratio \kappa [-]', 'FontSize', 11);
ylabel('Longitudinal Tire Force F_x [N]', 'FontSize', 11);
title('Baseline Longitudinal Tire Force vs. Slip Ratio (F_z = 4000 N)', 'FontSize', 12, 'FontWeight', 'bold');
xlim([-1.05, 1.05]);
ylim([-4500, 4500]);

figFile = fullfile(figDir, 'baseline_longitudinal_tire_force.png');
saveas(hFig, figFile);
close(hFig);
fprintf('Saved figure to: %s\n', figFile);

% 7. Save Numerical Sweep Data
csvFile = fullfile(dataDir, 'baseline_longitudinal_tire_force.csv');
dataTable = table(kappa, Fx, 'VariableNames', {'kappa', 'Fx_N'});
writetable(dataTable, csvFile);
fprintf('Saved sweep data to: %s\n', csvFile);
