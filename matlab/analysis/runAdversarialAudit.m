function runAdversarialAudit()
    logFile = 'adversarial_audit.log';
    fid = fopen(logFile, 'w');
    fprintf(fid, '=== ADVERSARIAL AUDIT ===\n');

    % 1. Parameters
    fprintf(fid, '--- 2. PARAMETER ISOLATION ---\n');
    params = roverParameters();
    cond_snow = roadSurfaceCondition('Snow');
    cond_chain = snowChainCondition();
    
    snow_p = cond_snow.pure_params;
    snow_c = cond_snow.combined_params;
    chain_p = cond_chain.pure_params;
    chain_c = cond_chain.combined_params;

    fprintf(fid, 'Snow pure: Bx=%.4f, Cx=%.4f, Dx=%.4f, Ex=%.4f, By=%.4f, Cy=%.4f, Dy=%.4f, Ey=%.4f\n', ...
        snow_p.Bx, snow_p.Cx, snow_p.Dx, snow_p.Ex, snow_p.By, snow_p.Cy, snow_p.Dy, snow_p.Ey);
    fprintf(fid, 'Chain pure: Bx=%.4f, Cx=%.4f, Dx=%.4f, Ex=%.4f, By=%.4f, Cy=%.4f, Dy=%.4f, Ey=%.4f\n', ...
        chain_p.Bx, chain_p.Cx, chain_p.Dx, chain_p.Ex, chain_p.By, chain_p.Cy, chain_p.Dy, chain_p.Ey);
        
    diff_p = cell2mat(struct2cell(chain_p)) - cell2mat(struct2cell(snow_p));
    diff_c = cell2mat(struct2cell(chain_c)) - cell2mat(struct2cell(snow_c));
    fprintf(fid, 'Total pure diff elements > 0: %d\n', sum(abs(diff_p)>1e-9));
    fprintf(fid, 'Total comb diff elements > 0: %d\n', sum(abs(diff_c)>1e-9));
    
    % 4. Results Recomputation
    fprintf(fid, '--- 4. RESULTS RECOMPUTATION ---\n');
    benchmark = runSnowChainBenchmark(params);

    % Acceleration
    t = benchmark.StraightAcceleration.Snow.time;
    state_s = benchmark.StraightAcceleration.Snow.state;
    diag_s = benchmark.StraightAcceleration.Snow.diagnostics;
    state_c = benchmark.StraightAcceleration.SnowChain.state;
    diag_c = benchmark.StraightAcceleration.SnowChain.diagnostics;
    
    fprintf(fid, 'Accel Final Speed (Snow): %.6f m/s\n', sqrt(state_s(end,4)^2 + state_s(end,5)^2));
    fprintf(fid, 'Accel Final Speed (Chain): %.6f m/s\n', sqrt(state_c(end,4)^2 + state_c(end,5)^2));
    fprintf(fid, 'Accel Peak Fx (Snow): %.6f N\n', max(diag_s.Fx_total));
    fprintf(fid, 'Accel Peak Fx (Chain): %.6f N\n', max(diag_c.Fx_total));

    % Braking
    b_s = benchmark.StraightBraking.Snow;
    b_c = benchmark.StraightBraking.SnowChain;
    fprintf(fid, 'Brake Final Speed (Snow): %.6f m/s\n', sqrt(b_s.state(end,4)^2 + b_s.state(end,5)^2));
    fprintf(fid, 'Brake Final Speed (Chain): %.6f m/s\n', sqrt(b_c.state(end,4)^2 + b_c.state(end,5)^2));
    
    dist_s = sum(sqrt(diff(b_s.state(:,1)).^2 + diff(b_s.state(:,2)).^2));
    dist_c = sum(sqrt(diff(b_c.state(:,1)).^2 + diff(b_c.state(:,2)).^2));
    euc_s = sqrt((b_s.state(end,1)-b_s.state(1,1))^2 + (b_s.state(end,2)-b_s.state(1,2))^2);
    euc_c = sqrt((b_c.state(end,1)-b_c.state(1,1))^2 + (b_c.state(end,2)-b_c.state(1,2))^2);
    
    fprintf(fid, 'Brake Path Dist (Snow): %.6f m\n', dist_s);
    fprintf(fid, 'Brake Path Dist (Chain): %.6f m\n', dist_c);
    fprintf(fid, 'Brake Euclidean Dist (Snow): %.6f m\n', euc_s);
    fprintf(fid, 'Brake Euclidean Dist (Chain): %.6f m\n', euc_c);
    
    fprintf(fid, 'Brake Final Y (Snow): %.6e m\n', b_s.state(end,2));
    fprintf(fid, 'Brake Final Y (Chain): %.6e m\n', b_c.state(end,2));

    % Cornering
    c_s = benchmark.ConstantCornering.Snow;
    c_c = benchmark.ConstantCornering.SnowChain;
    
    fprintf(fid, 'Corner Peak Fy (Snow): %.6f N\n', max(abs(c_s.diagnostics.Fy_total)));
    fprintf(fid, 'Corner Peak Fy (Chain): %.6f N\n', max(abs(c_c.diagnostics.Fy_total)));
    fprintf(fid, 'Corner Peak Yaw Rate (Snow): %.6f rad/s\n', max(abs(c_s.state(:,6))));
    fprintf(fid, 'Corner Peak Yaw Rate (Chain): %.6f rad/s\n', max(abs(c_c.state(:,6))));
    
    lat_dev_s = c_s.state(end, 2);
    lat_dev_c = c_c.state(end, 2);
    fprintf(fid, 'Corner Final Lat Dev Y (Snow): %.6f m\n', lat_dev_s);
    fprintf(fid, 'Corner Final Lat Dev Y (Chain): %.6f m\n', lat_dev_c);
    
    fprintf(fid, 'Corner FL vs FR max Fy (Snow): %.6f vs %.6f N\n', max(abs(c_s.diagnostics.Fy_wheel(:,1))), max(abs(c_s.diagnostics.Fy_wheel(:,2))));
    fprintf(fid, 'Corner RL vs RR max Fy (Snow): %.6f vs %.6f N\n', max(abs(c_s.diagnostics.Fy_wheel(:,3))), max(abs(c_s.diagnostics.Fy_wheel(:,4))));
    
    % Sanity Checks
    m = params.m; g = 9.81; Fz_total = m * g;
    fprintf(fid, 'Mass: %.2f kg, Total Fz: %.2f N\n', m, Fz_total);
    fprintf(fid, 'Snow Peak Fx/Fz (Accel): %.4f\n', max(diag_s.Fx_total) / Fz_total);
    fprintf(fid, 'Chain Peak Fx/Fz (Accel): %.4f\n', max(diag_c.Fx_total) / Fz_total);
    fprintf(fid, 'Snow Peak Fy/Fz (Corner): %.4f\n', max(abs(c_s.diagnostics.Fy_total)) / Fz_total);
    fprintf(fid, 'Chain Peak Fy/Fz (Corner): %.4f\n', max(abs(c_c.diagnostics.Fy_total)) / Fz_total);
    
    fclose(fid);
end
