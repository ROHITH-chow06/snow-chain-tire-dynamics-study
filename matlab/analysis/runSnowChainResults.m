function runSnowChainResults()
% RUNSNOWCHAINRESULTS Generates final Phase 3K figures and numerical table.
%
%   Runs the established benchmark and creates clean, scientifically grounded
%   visualizations of the simulation data. Does not introduce new physics.

    fprintf('============================================================\n');
    fprintf('  Phase 3K: Snow vs SnowChain Benchmark Results\n');
    fprintf('============================================================\n\n');

    % Run the established benchmark
    params = roverParameters();
    benchmark = runSnowChainBenchmark(params);

    % Define simple, clean colors for plots
    c_snow = [0 0.4470 0.7410];       % Blue
    c_chain = [0.8500 0.3250 0.0980]; % Orange

    % Create directory for figures if it doesn't exist
    fig_dir = fullfile(pwd, 'results', 'figures');
    if ~exist(fig_dir, 'dir')
        mkdir(fig_dir);
    end

    % ------------------------------------------------------------------
    % FIGURE 1 - Braking Distance
    % ------------------------------------------------------------------
    fig1 = figure('Name', 'Braking Distance', 'Color', 'w', 'Visible', 'off');
    
    dist_snow = sqrt((benchmark.StraightBraking.Snow.handling.final_X - benchmark.StraightBraking.Snow.state(1,1))^2 + ...
                     (benchmark.StraightBraking.Snow.handling.final_Y - benchmark.StraightBraking.Snow.state(1,2))^2);
    dist_chain = sqrt((benchmark.StraightBraking.SnowChain.handling.final_X - benchmark.StraightBraking.SnowChain.state(1,1))^2 + ...
                      (benchmark.StraightBraking.SnowChain.handling.final_Y - benchmark.StraightBraking.SnowChain.state(1,2))^2);
                      
    b = bar([1, 2], [dist_snow, dist_chain], 'FaceColor', 'flat');
    b.CData(1,:) = c_snow;
    b.CData(2,:) = c_chain;
    
    set(gca, 'XTick', [1, 2], 'XTickLabel', {'Snow', 'SnowChain'});
    ylabel('Braking Distance (m)');
    title('Straight Braking Distance Comparison');
    grid on;
    
    % Add values on top of bars
    text(1, dist_snow + 0.1, sprintf('%.3f m', dist_snow), 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    text(2, dist_chain + 0.1, sprintf('%.3f m', dist_chain), 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    ylim([0, max(dist_snow, dist_chain) * 1.2]);
    
    exportgraphics(fig1, fullfile(fig_dir, '3K_braking_distance.png'), 'Resolution', 300);

    % ------------------------------------------------------------------
    % FIGURE 2 - Longitudinal Tire Force vs Slip (Fx vs kappa)
    % ------------------------------------------------------------------
    % We use the actual simulation time-history data for the Front-Left (FL) wheel.
    fig2 = figure('Name', 'Longitudinal Force vs Slip', 'Color', 'w', 'Visible', 'off');
    
    kappa_snow = benchmark.StraightAcceleration.Snow.diagnostics.kappa(:,1);
    Fx_snow = benchmark.StraightAcceleration.Snow.diagnostics.Fx_wheel(:,1);
    
    kappa_chain = benchmark.StraightAcceleration.SnowChain.diagnostics.kappa(:,1);
    Fx_chain = benchmark.StraightAcceleration.SnowChain.diagnostics.Fx_wheel(:,1);
    
    plot(kappa_snow, Fx_snow, 'LineWidth', 2, 'Color', c_snow);
    hold on;
    plot(kappa_chain, Fx_chain, 'LineWidth', 2, 'Color', c_chain);
    
    xlabel('Longitudinal Slip Ratio, \kappa');
    ylabel('Longitudinal Tire Force, F_x (N)');
    title('Dynamic Longitudinal Force-vs-Slip Trajectory (FL Wheel)');
    legend('Snow (Baseline)', 'SnowChain (Benchmark)', 'Location', 'Best');
    grid on;
    
    exportgraphics(fig2, fullfile(fig_dir, '3K_fx_vs_kappa.png'), 'Resolution', 300);

    % ------------------------------------------------------------------
    % FIGURE 3 - Cornering Trajectory (X vs Y)
    % ------------------------------------------------------------------
    fig3 = figure('Name', 'Cornering Trajectory', 'Color', 'w', 'Visible', 'off');
    
    X_snow = benchmark.ConstantCornering.Snow.state(:,1);
    Y_snow = benchmark.ConstantCornering.Snow.state(:,2);
    
    X_chain = benchmark.ConstantCornering.SnowChain.state(:,1);
    Y_chain = benchmark.ConstantCornering.SnowChain.state(:,2);
    
    plot(X_snow, Y_snow, 'LineWidth', 2, 'Color', c_snow);
    hold on;
    plot(X_chain, Y_chain, 'LineWidth', 2, 'Color', c_chain);
    
    xlabel('Global X Position (m)');
    ylabel('Global Y Position (m)');
    title('Constant Cornering Trajectory Overlay');
    legend('Snow', 'SnowChain', 'Location', 'NorthWest');
    grid on;
    axis equal;
    
    exportgraphics(fig3, fullfile(fig_dir, '3K_cornering_trajectory.png'), 'Resolution', 300);
    
    % ------------------------------------------------------------------
    % FIGURE 4 - Force Utilization (Fx vs Fy)
    % ------------------------------------------------------------------
    % This shows how the benchmark modified longitudinal boundary affects 
    % combined-slip behavior during cornering for the FL wheel.
    fig4 = figure('Name', 'Force Utilization (Fx vs Fy)', 'Color', 'w', 'Visible', 'off');
    
    Fx_snow = benchmark.ConstantCornering.Snow.diagnostics.Fx_wheel(:,1);
    Fy_snow = benchmark.ConstantCornering.Snow.diagnostics.Fy_wheel(:,1);
    
    Fx_chain = benchmark.ConstantCornering.SnowChain.diagnostics.Fx_wheel(:,1);
    Fy_chain = benchmark.ConstantCornering.SnowChain.diagnostics.Fy_wheel(:,1);
    
    plot(Fy_snow, Fx_snow, 'LineWidth', 1.5, 'Color', c_snow);
    hold on;
    plot(Fy_chain, Fx_chain, 'LineWidth', 1.5, 'Color', c_chain);
    
    xlabel('Lateral Tire Force, F_y (N)');
    ylabel('Longitudinal Tire Force, F_x (N)');
    title('Dynamic Force Utilization Trace (FL Wheel, Cornering)');
    legend('Snow', 'SnowChain', 'Location', 'Best');
    grid on;
    
    exportgraphics(fig4, fullfile(fig_dir, '3K_force_utilization.png'), 'Resolution', 300);

    % ------------------------------------------------------------------
    % PRINT RESULTS TABLE
    % ------------------------------------------------------------------
    fprintf('%-30s | %-12s | %-12s | %-15s | %-10s\n', 'Metric', 'Snow', 'SnowChain', 'Abs Diff', '%% Diff');
    fprintf('%s\n', repmat('-', 1, 85));
    
    % Straight Acceleration
    v_snow = benchmark.StraightAcceleration.Snow.handling.final_speed;
    v_chain = benchmark.StraightAcceleration.SnowChain.handling.final_speed;
    fprintf('%-30s | %12.3f | %12.3f | %15.3f | %9.1f%%\n', ...
        '[Accel] Final Speed (m/s)', v_snow, v_chain, v_chain - v_snow, ((v_chain - v_snow)/v_snow)*100);
        
    fx_snow = benchmark.StraightAcceleration.Snow.handling.peak_Fx;
    fx_chain = benchmark.StraightAcceleration.SnowChain.handling.peak_Fx;
    fprintf('%-30s | %12.3f | %12.3f | %15.3f | %9.1f%%\n', ...
        '[Accel] Peak Fx (N)', fx_snow, fx_chain, fx_chain - fx_snow, ((fx_chain - fx_snow)/fx_snow)*100);

    % Straight Braking
    fprintf('%-30s | %12.3f | %12.3f | %15.3f | %9.1f%%\n', ...
        '[Brake] Distance (m)', dist_snow, dist_chain, dist_chain - dist_snow, ((dist_chain - dist_snow)/dist_snow)*100);

    % Constant Cornering
    fy_snow = benchmark.ConstantCornering.Snow.handling.peak_Fy;
    fy_chain = benchmark.ConstantCornering.SnowChain.handling.peak_Fy;
    fprintf('%-30s | %12.3f | %12.3f | %15.3f | %9.3f%%\n', ...
        '[Turn] Peak Fy (N)', fy_snow, fy_chain, fy_chain - fy_snow, ((fy_chain - fy_snow)/fy_snow)*100);
        
    r_snow = benchmark.ConstantCornering.Snow.handling.peak_abs_r;
    r_chain = benchmark.ConstantCornering.SnowChain.handling.peak_abs_r;
    fprintf('%-30s | %12.3f | %12.3f | %15.3f | %9.1f%%\n', ...
        '[Turn] Peak Yaw Rate (rad/s)', r_snow, r_chain, r_chain - r_snow, ((r_chain - r_snow)/r_snow)*100);
        
    dev_snow = benchmark.ConstantCornering.Snow.handling.final_Y;
    dev_chain = benchmark.ConstantCornering.SnowChain.handling.final_Y;
    fprintf('%-30s | %12.3f | %12.3f | %15.3f | %9.1f%%\n', ...
        '[Turn] Final Lat Dev (m)', dev_snow, dev_chain, dev_chain - dev_snow, ((dev_chain - dev_snow)/dev_snow)*100);

    fprintf('\nFigures saved to results/figures/.\n');
end
