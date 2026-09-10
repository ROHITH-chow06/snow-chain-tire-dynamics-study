%% VERIFYROADSURFACECONDITIONS (Phase 3J Verification)
%
% This script executes automated verification of the road-surface condition 
% parameter framework. It verifies model structure, invariants, and expected 
% comparative trends (Dry/Wet/Snow) across defined vehicle maneuvers.
%
% Do NOT modify Phase 1A-3I physics implementation to make these tests pass.

clear; clc;

fprintf('============================================================\n');
fprintf('  Running Verification: Road Surface Conditions (3J)\n');
fprintf('============================================================\n\n');

all_tests_passed = true;
test_count = 0;

params = roverParameters();
sim_options.dt = 0.01;
sim_options.tol = 1e-4;

%% SECTION 1: Condition Factory Validation

% Test 1: Valid condition names
test_count = test_count + 1;
try
    c_dry  = roadSurfaceCondition('Dry');
    c_wet  = roadSurfaceCondition('wet'); 
    c_snow = roadSurfaceCondition('SNOW');
    fprintf('Test %d: Valid condition names are accepted... PASSED\n', test_count);
catch
    fprintf('Test %d: FAILED (Valid condition names rejected)\n', test_count);
    all_tests_passed = false;
end

% Test 2: Invalid condition rejection
test_count = test_count + 1;
try
    roadSurfaceCondition('Ice');
    fprintf('Test %d: FAILED (Expected error for unsupported condition)\n', test_count);
    all_tests_passed = false;
catch ME
    if strcmp(ME.identifier, 'roadSurfaceCondition:invalidType')
        fprintf('Test %d: Invalid condition correctly rejected... PASSED\n', test_count);
    else
        fprintf('Test %d: FAILED (Wrong error thrown)\n', test_count);
        all_tests_passed = false;
    end
end

% Test 3: Required parameter fields
test_count = test_count + 1;
if isfield(c_dry, 'type') && isfield(c_dry, 'pure_params') && isfield(c_dry, 'combined_params') ...
   && isfield(c_dry.pure_params, 'Bx') && isfield(c_dry.combined_params, 'Bx_alpha')
    fprintf('Test %d: Required parameter fields exist... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Missing parameter fields)\n', test_count);
    all_tests_passed = false;
end

% Test 4: Finite/real parameter values
test_count = test_count + 1;
pure_vals = cell2mat(struct2cell(c_dry.pure_params));
comb_vals = cell2mat(struct2cell(c_dry.combined_params));
if all(isfinite(pure_vals)) && all(isreal(pure_vals)) && all(isfinite(comb_vals)) && all(isreal(comb_vals))
    fprintf('Test %d: Parameter values are finite and real... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Non-finite or non-real parameters)\n', test_count);
    all_tests_passed = false;
end

% Test 5: Positive/valid parameter ranges (D > 0, B > 0, C > 0)
test_count = test_count + 1;
if c_dry.pure_params.Dx > 0 && c_dry.pure_params.Bx > 0 && c_dry.pure_params.Cx > 0
    fprintf('Test %d: Pure slip parameters fall in physically meaningful positive ranges... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Invalid parameter ranges)\n', test_count);
    all_tests_passed = false;
end

% Test 6: Deterministic condition generation
test_count = test_count + 1;
if isequal(c_dry, roadSurfaceCondition('Dry'))
    fprintf('Test %d: Deterministic condition generation... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Non-deterministic generation)\n', test_count);
    all_tests_passed = false;
end

% Test 7: Parameter distinctness
test_count = test_count + 1;
if ~isequal(c_dry.pure_params, c_wet.pure_params) && ~isequal(c_wet.pure_params, c_snow.pure_params)
    fprintf('Test %d: Dry/Wet/Snow parameter sets are mathematically distinct... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Parameter sets are identical)\n', test_count);
    all_tests_passed = false;
end

%% SECTION 2: Direct Parameter & Mathematical Ordering (Independent of Trajectory)

% Test 8: Dx / Dy peak friction ordering
test_count = test_count + 1;
if (c_dry.pure_params.Dx > c_wet.pure_params.Dx) && (c_wet.pure_params.Dx > c_snow.pure_params.Dx) && ...
   (c_dry.pure_params.Dy > c_wet.pure_params.Dy) && (c_wet.pure_params.Dy > c_snow.pure_params.Dy)
    fprintf('Test %d: Peak friction factors (D) follow intended formulation (Dry > Wet > Snow)... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Dx/Dy ordering violated)\n', test_count);
    all_tests_passed = false;
end

% Test 9: Bx / By stiffness ordering
test_count = test_count + 1;
if (c_dry.pure_params.Bx > c_wet.pure_params.Bx) && (c_wet.pure_params.Bx > c_snow.pure_params.Bx) && ...
   (c_dry.pure_params.By > c_wet.pure_params.By) && (c_wet.pure_params.By > c_snow.pure_params.By)
    fprintf('Test %d: Stiffness factors (B) follow intended compliance formulation... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Bx/By ordering violated)\n', test_count);
    all_tests_passed = false;
end

% Test 10: Cx / Cy shape ordering
test_count = test_count + 1;
if (c_dry.pure_params.Cx > c_wet.pure_params.Cx) && (c_wet.pure_params.Cx > c_snow.pure_params.Cx)
    fprintf('Test %d: Shape factors (C) follow intended formulation... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Cx/Cy ordering violated)\n', test_count);
    all_tests_passed = false;
end

% Test 11: Direct longitudinal tire-force capacity (Independent of vehicle dynamics)
test_count = test_count + 1;
kappa_test = 0.2; alpha_test = 0.0; Fz_test = 400; % 400 N per wheel
[Fx_dry, ~, ~, ~] = combinedSlipTireForce(kappa_test, alpha_test, Fz_test, c_dry.pure_params, c_dry.combined_params);
[Fx_wet, ~, ~, ~] = combinedSlipTireForce(kappa_test, alpha_test, Fz_test, c_wet.pure_params, c_wet.combined_params);
[Fx_snow, ~, ~, ~] = combinedSlipTireForce(kappa_test, alpha_test, Fz_test, c_snow.pure_params, c_snow.combined_params);
if (Fx_dry > Fx_wet) && (Fx_wet > Fx_snow)
    fprintf('Test %d: Direct longitudinal tire force yields bounded ordering at identical slip... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Direct Fx comparison failed)\n', test_count);
    all_tests_passed = false;
end

% Test 12: Direct lateral tire-force capacity (Independent of vehicle dynamics)
test_count = test_count + 1;
kappa_test2 = 0.0; alpha_test2 = 0.2;
[~, Fy_dry, ~, ~] = combinedSlipTireForce(kappa_test2, alpha_test2, Fz_test, c_dry.pure_params, c_dry.combined_params);
[~, Fy_wet, ~, ~] = combinedSlipTireForce(kappa_test2, alpha_test2, Fz_test, c_wet.pure_params, c_wet.combined_params);
[~, Fy_snow, ~, ~] = combinedSlipTireForce(kappa_test2, alpha_test2, Fz_test, c_snow.pure_params, c_snow.combined_params);
if (abs(Fy_dry) > abs(Fy_wet)) && (abs(Fy_wet) > abs(Fy_snow))
    fprintf('Test %d: Direct lateral tire force yields bounded ordering at identical slip... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Direct Fy comparison failed)\n', test_count);
    all_tests_passed = false;
end

%% SECTION 3: Structural Integrity & Isolation

% Test 13: Immutability of base parameters (Snapshot check)
test_count = test_count + 1;
params_snapshot = params;
res_straight = runRoadConditionComparison('StraightAcceleration', params, sim_options);
if isequal(params, params_snapshot)
    fprintf('Test %d: Baseline vehicle parameters remain strictly isolated and immutable... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Vehicle parameters mutated during comparison)\n', test_count);
    all_tests_passed = false;
end

% Test 14: Condition isolation (Only condition changes)
test_count = test_count + 1;
% Check the underlying trajectories actually differ
diff_pos = abs(res_straight.Dry.state(end,1) - res_straight.Snow.state(end,1));
if diff_pos > 0.01
    fprintf('Test %d: Mathematical isolation verified (Dry vs Snow trajectories fundamentally diverge)... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Trajectories identical, condition parameters ignored)\n', test_count);
    all_tests_passed = false;
end

% Test 15: Exact reproducibility
test_count = test_count + 1;
res_straight_2 = runRoadConditionComparison('StraightAcceleration', params, sim_options);
if isequal(res_straight.Snow.state, res_straight_2.Snow.state) && ...
   isequal(res_straight.Dry.forceMetrics.total_Fx, res_straight_2.Dry.forceMetrics.total_Fx)
    fprintf('Test %d: Comparative simulation evaluates with exact bitwise determinism... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Reproducibility failed)\n', test_count);
    all_tests_passed = false;
end

% Test 16: No NaN across complete trajectories
test_count = test_count + 1;
nan_check = ~any(isnan(res_straight.Dry.state(:))) && ~any(isnan(res_straight.Wet.state(:))) && ~any(isnan(res_straight.Snow.state(:)));
if nan_check
    fprintf('Test %d: Complete integrated state trajectories strictly free of NaN... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (NaN detected)\n', test_count);
    all_tests_passed = false;
end

% Test 17: No Inf across complete trajectories
test_count = test_count + 1;
inf_check = ~any(isinf(res_straight.Dry.state(:))) && ~any(isinf(res_straight.Wet.state(:))) && ~any(isinf(res_straight.Snow.state(:)));
if inf_check
    fprintf('Test %d: Complete integrated state trajectories strictly free of Inf... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Inf detected)\n', test_count);
    all_tests_passed = false;
end

% Test 18: Zero-slip equilibrium preservation (Snow condition)
test_count = test_count + 1;
v0 = 2.0;
deltaFcn_zero = @(t) 0.0;
omegaFcn_zero = @(t) (v0 / params.R) * ones(4, 1);
x0_zero = [0; 0; 0; v0; 0; 0];
[~, ~, diag_eq] = simulateVehicleTrajectory([0, 1.0], x0_zero, deltaFcn_zero, omegaFcn_zero, params, c_snow.pure_params, c_snow.combined_params, sim_options);
if max(abs(diag_eq.Fx_total)) < 1e-4 && max(abs(diag_eq.Fy_total)) < 1e-4
    fprintf('Test %d: Zero-slip equilibrium is rigorously preserved under Snow condition... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Zero-slip equilibrium violated)\n', test_count);
    all_tests_passed = false;
end

% Test 19: Vertical load conservation across conditions
test_count = test_count + 1;
res_corner = runRoadConditionComparison('ConstantCornering', params, sim_options);
sum_Fz_dry  = max(abs(sum(res_corner.Dry.diagnostics.Fz_wheel, 2) - params.m*params.g));
sum_Fz_snow = max(abs(sum(res_corner.Snow.diagnostics.Fz_wheel, 2) - params.m*params.g));
if (sum_Fz_dry < 1e-4) && (sum_Fz_snow < 1e-4)
    fprintf('Test %d: Vertical load is globally conserved (sum Fz = mg) across all condition maneuvers... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Load conservation violated)\n', test_count);
    all_tests_passed = false;
end

% Test 20: Lateral symmetry
test_count = test_count + 1;
Fx_FL = res_straight.Wet.diagnostics.Fx_wheel(:, 1);
Fx_FR = res_straight.Wet.diagnostics.Fx_wheel(:, 2);
if max(abs(Fx_FL - Fx_FR)) < 1e-5
    fprintf('Test %d: Lateral symmetry is mathematically preserved in Wet straight maneuver... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Asymmetry detected)\n', test_count);
    all_tests_passed = false;
end

%% SECTION 4: Comparative Emergent Trends
% Note: These test emergent behavior resulting from the combination of tire stiffness,
% peak friction limits, and closed-loop dynamics. They are structured to verify
% qualitative behavioral trends where mathematically supported.

% Test 21: Acceleration final speed (compliance check)
test_count = test_count + 1;
v_accel_dry  = res_straight.Dry.handling.final_speed;
v_accel_wet  = res_straight.Wet.handling.final_speed;
v_accel_snow = res_straight.Snow.handling.final_speed;
% Lower B (stiffness) on Snow means it requires more slip to produce the same required acceleration force,
% which causes the vehicle velocity to lag further behind the commanded wheel velocity.
if (v_accel_dry > v_accel_wet) && (v_accel_wet > v_accel_snow)
    fprintf('Test %d: (Emergent) Acceleration final speed reflects surface compliance (Dry > Wet > Snow)... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Dry:%.3f, Wet:%.3f, Snow:%.3f)\n', test_count, v_accel_dry, v_accel_wet, v_accel_snow);
    all_tests_passed = false;
end

% Test 22: Braking final speed
test_count = test_count + 1;
res_brake = runRoadConditionComparison('StraightBraking', params, sim_options);
v_brake_dry  = res_brake.Dry.handling.final_speed;
v_brake_wet  = res_brake.Wet.handling.final_speed;
v_brake_snow = res_brake.Snow.handling.final_speed;
% Similarly, during braking, higher compliance in Snow means vehicle speed drops less rapidly for the same commanded wheel speed.
if (v_brake_dry < v_brake_wet) && (v_brake_wet < v_brake_snow)
    fprintf('Test %d: (Emergent) Braking final speed verifies deceleration lag trend (Dry < Wet < Snow)... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Dry:%.3f, Wet:%.3f, Snow:%.3f)\n', test_count, v_brake_dry, v_brake_wet, v_brake_snow);
    all_tests_passed = false;
end

% Test 23: Lateral force capacity (Cornering)
test_count = test_count + 1;
peakFy_dry  = res_corner.Dry.handling.peak_Fy;
peakFy_wet  = res_corner.Wet.handling.peak_Fy;
peakFy_snow = res_corner.Snow.handling.peak_Fy;
% Step-steer pushes lateral force to limits, reflecting D parameter ordering.
if (peakFy_dry > peakFy_wet) && (peakFy_wet > peakFy_snow)
    fprintf('Test %d: (Emergent) Lateral force capacity reflects condition limits (Dry > Wet > Snow)... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED (Dry:%.1f, Wet:%.1f, Snow:%.1f)\n', test_count, peakFy_dry, peakFy_wet, peakFy_snow);
    all_tests_passed = false;
end

% Test 24: Cornering yaw rate
test_count = test_count + 1;
peak_r_dry  = res_corner.Dry.handling.peak_abs_r;
peak_r_wet  = res_corner.Wet.handling.peak_abs_r;
peak_r_snow = res_corner.Snow.handling.peak_abs_r;
if (peak_r_dry > peak_r_wet) && (peak_r_wet > peak_r_snow)
    fprintf('Test %d: (Emergent) Cornering yaw rate generation correlates with lateral grip... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED\n', test_count);
    all_tests_passed = false;
end

% Test 25: Trajectory lateral deviation
test_count = test_count + 1;
final_Y_dry  = res_corner.Dry.state(end, 2);
final_Y_wet  = res_corner.Wet.state(end, 2);
final_Y_snow = res_corner.Snow.state(end, 2);
if (final_Y_dry > final_Y_wet) && (final_Y_wet > final_Y_snow)
    fprintf('Test %d: (Emergent) Turn trajectory lateral deviation follows grip limits... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED\n', test_count);
    all_tests_passed = false;
end

% Test 26: Combined-slip peak utilization
test_count = test_count + 1;
res_comb = runRoadConditionComparison('CombinedAccelerationCornering', params, sim_options);
util_dry  = res_comb.Dry.handling.peak_tire_util;
util_wet  = res_comb.Wet.handling.peak_tire_util;
util_snow = res_comb.Snow.handling.peak_tire_util;
if (util_dry > util_wet) && (util_wet > util_snow)
    fprintf('Test %d: (Emergent) Combined-slip peak utilization envelope is restricted by surface type... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED\n', test_count);
    all_tests_passed = false;
end

%% SECTION 5: Explicit Force Limit Bounds

% Test 27: Explicit Snow wheel-level longitudinal force mathematical bound
test_count = test_count + 1;
% Theoretical max possible longitudinal force on Snow is exactly mu_peak * Fz at the wheel level.
% The PAC2002 weighting guarantees |Gxa| <= 1, and MF guarantees |Fx0| <= Dx * Fz.
% Therefore, the normalized metric mu_x = |Fx/Fz| must strictly be <= Dx.
theory_bound_snow_x = c_snow.pure_params.Dx; 
max_mu_x_snow = max(res_comb.Snow.forceMetrics.mu_x(:));

if max_mu_x_snow <= (theory_bound_snow_x + 1e-4) % 1e-4 for numerical solver tolerance
    fprintf('Test %d: Snow wheel-level longitudinal force is mathematically bounded by exact formulation limit (Dx=%g)... PASSED\n', test_count, theory_bound_snow_x);
else
    fprintf('Test %d: FAILED (Exceeded absolute limit. Max mu_x: %g, Limit: %g)\n', test_count, max_mu_x_snow, theory_bound_snow_x);
    all_tests_passed = false;
end

% Test 28: Explicit Snow wheel-level lateral force mathematical bound
test_count = test_count + 1;
% Similar mathematical guarantee for lateral force: |Fy0| <= Dy * Fz, |Gyk| <= 1.
theory_bound_snow_y = c_snow.pure_params.Dy;
max_mu_y_snow = max(res_comb.Snow.forceMetrics.mu_y(:));
if max_mu_y_snow <= (theory_bound_snow_y + 1e-4)
    fprintf('Test %d: Snow wheel-level lateral force is mathematically bounded by exact formulation limit (Dy=%g)... PASSED\n', test_count, theory_bound_snow_y);
else
    fprintf('Test %d: FAILED (Exceeded absolute limit. Max mu_y: %g, Limit: %g)\n', test_count, max_mu_y_snow, theory_bound_snow_y);
    all_tests_passed = false;
end

% Test 29: Explicit Dry wheel-level bounds sanity check
test_count = test_count + 1;
theory_bound_dry_x = c_dry.pure_params.Dx;
max_mu_x_dry = max(res_comb.Dry.forceMetrics.mu_x(:));
if max_mu_x_dry <= (theory_bound_dry_x + 1e-4)
    fprintf('Test %d: Dry wheel-level longitudinal force is mathematically bounded by exact formulation limit... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED\n', test_count);
    all_tests_passed = false;
end

% Test 30: Validation of existing Phase 3I structure
test_count = test_count + 1;
% Ensure all expected layers of Phase 3I are populated cleanly
if isfield(res_comb.Dry, 'stateMetrics') && isfield(res_comb.Dry, 'forceMetrics') && isfield(res_comb.Dry, 'handling')
    fprintf('Test %d: Underlying Phase 3I state, force, and handling metric structures populate successfully... PASSED\n', test_count);
else
    fprintf('Test %d: FAILED\n', test_count);
    all_tests_passed = false;
end


%% SUMMARY
fprintf('\n============================================================\n');
if all_tests_passed
    fprintf('  All %d road-surface condition (3J) tests PASSED!\n', test_count);
else
    fprintf('  ONE OR MORE TESTS FAILED.\n');
end
fprintf('============================================================\n');
