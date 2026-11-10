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

    % Create white-background figure
    fig = figure('Name', 'Snow Baseline Longitudinal Tire Force', ...
        'Color', 'w', ...
        'Position', [100, 100, 1000, 650], ...
        'Visible', 'off', ...
        'InvertHardcopy', 'off');

    % Create axes and explicitly use black-on-white styling
    ax = axes('Parent', fig);
    set(ax, ...
        'Color', 'w', ...
        'XColor', 'k', ...
        'YColor', 'k', ...
        'GridColor', 'k', ...
        'GridAlpha', 0.15, ...
        'LineWidth', 1.0, ...
        'FontSize', 12);

    % Blue tire-force curve
    plot(ax, kappa, Fx, ...
        'LineWidth', 2.2, ...
        'Color', [0.0, 0.4470, 0.7410]);

    hold(ax, 'on');

    % Red peak marker
    plot(ax, peakDrivingSlip, peakDrivingForce, 'ro', ...
        'MarkerFaceColor', 'r', ...
        'MarkerSize', 7);

    % Black reference axes
    xline(ax, 0, 'k:', 'LineWidth', 0.9);
    yline(ax, 0, 'k--', 'LineWidth', 0.9);

    hold(ax, 'off');

    grid(ax, 'on');
    box(ax, 'on');
    xlim(ax, [-1, 1]);

    xlabel(ax, 'Longitudinal Slip Ratio, \kappa [-]', ...
        'Color', 'k');

    ylabel(ax, 'Longitudinal Tire Force, F_x [N]', ...
        'Color', 'k');

    title(ax, {'Snow Baseline Pure Longitudinal Tire Force', ...
        sprintf('F_z = %.2f N per wheel (static equal-share load)', Fz)}, ...
        'Color', 'k');

    lgd = legend(ax, 'Snow baseline', 'Peak driving force', ...
        'Location', 'best');

    set(lgd, ...
        'TextColor', 'k', ...
        'Color', 'w', ...
        'EdgeColor', 'k');

    % Export high-resolution image with a white background
    pngFile = fullfile(figDir, ...
        'snow_baseline_longitudinal_tire_force.png');

    exportgraphics(fig, pngFile, ...
        'Resolution', 300, ...
        'BackgroundColor', 'white');

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
