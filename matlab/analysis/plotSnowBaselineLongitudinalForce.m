function plotSnowBaselineLongitudinalForce()
% PLOTSNOWBASELINELONGITUDINALFORCE
% Generate the current Snow baseline pure-longitudinal Magic Formula curve.
%
% Uses:
%   roadSurfaceCondition('Snow') for Bx, Cx, Dx, Ex
%   roverParameters() for vehicle mass and gravity
%   Fz = m*g/4 as an equal-share static load per wheel
%
% This is a quasi-steady, pure-slip characterization (alpha = 0), not a
% transient simulation trace, combined-slip envelope, or measured tire test.

    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(fullfile(rootDir, 'matlab', 'tire'));
    addpath(fullfile(rootDir, 'matlab', 'vehicle'));

    snow = roadSurfaceCondition('Snow');
    vehicle = roverParameters();
    p = snow.pure_params;

    Fz = vehicle.m * vehicle.g / 4.0;
    kappa = linspace(-1.0, 1.0, 2001)';

    Fx = magicFormulaLongitudinal( ...
        kappa, Fz, p.Bx, p.Cx, p.Dx, p.Ex);

    positiveMask = kappa >= 0;
    positiveForce = Fx(positiveMask);
    positiveSlip = kappa(positiveMask);
    [peakDrivingForce, peakIndex] = max(positiveForce);
    peakDrivingSlip = positiveSlip(peakIndex);

    figDir = fullfile(rootDir, 'results', 'reference_snow_run', 'figures');
    dataDir = fullfile(rootDir, 'results', 'reference_snow_run', 'data');
    if ~exist(figDir, 'dir'), mkdir(figDir); end
    if ~exist(dataDir, 'dir'), mkdir(dataDir); end

    fig = figure('Name', 'Snow Baseline Longitudinal Tire Force', ...
        'Color', 'w', 'Position', [100, 100, 1000, 650], 'Visible', 'off');

    plot(kappa, Fx, 'LineWidth', 2.2, 'Color', [0.0, 0.4470, 0.7410]);
    hold on;
    plot(peakDrivingSlip, peakDrivingForce, 'ro', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 7);
    xline(0, 'k:', 'LineWidth', 0.9);
    yline(0, 'k--', 'LineWidth', 0.9);
    hold off;

    grid on;
    box on;
    xlim([-1, 1]);
    xlabel('Longitudinal Slip Ratio, \kappa [-]');
    ylabel('Longitudinal Tire Force, F_x [N]');
    title({'Snow Baseline Pure Longitudinal Tire Force', ...
        sprintf('F_z = %.2f N per wheel (static equal-share load)', Fz)});
    legend('Snow baseline', 'Peak driving force', 'Location', 'best');

    pngFile = fullfile(figDir, 'snow_baseline_longitudinal_tire_force.png');
    exportgraphics(fig, pngFile, 'Resolution', 300);
    close(fig);

    forceTable = table(kappa, Fx, ...
        'VariableNames', {'kappa', 'Fx_Snow_N'});
    csvFile = fullfile(dataDir, 'snow_baseline_longitudinal_tire_force.csv');
    writetable(forceTable, csvFile);

    fprintf('Snow baseline pure-slip force characterization completed.\n');
    fprintf('Snow coefficients: Bx=%.2f, Cx=%.2f, Dx=%.2f, Ex=%.2f\n', ...
        p.Bx, p.Cx, p.Dx, p.Ex);
    fprintf('Vehicle mass: %.2f kg; static equal-share Fz: %.2f N\n', ...
        vehicle.m, Fz);
    fprintf('Peak driving force in sweep: %.3f N at kappa=%.4f\n', ...
        peakDrivingForce, peakDrivingSlip);
    fprintf('Saved PNG: %s\n', pngFile);
    fprintf('Saved CSV: %s\n', csvFile);
end
