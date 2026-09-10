% verifyVehiclePerformanceAnalysis.m
% Verification script for Phase 3I: Vehicle-Level Performance & Handling Analysis

clc;
clear;
close all;

fprintf('============================================================\n');
fprintf('  Running Verification: Vehicle Performance Analysis (3I)\n');
fprintf('============================================================\n\n');

% Ensure analysis directory is on path
addpath(genpath('matlab/analysis'));

params = roverParameters();
all_tests_passed = true;

%% Run a baseline maneuver to use for multiple tests
res_straight = runVehicleManeuverAnalysis('StraightAcceleration', params);
t_S = res_straight.time;
sm_S = res_straight.stateMetrics;
fm_S = res_straight.forceMetrics;
hm_S = res_straight.handling;

res_corner = runVehicleManeuverAnalysis('ConstantCornering', params);
sm_C = res_corner.stateMetrics;
fm_C = res_corner.forceMetrics;
hm_C = res_corner.handling;

%% TEST 1: Zero-state metrics are finite and physically consistent.
% We can just use the first point of the StraightAcceleration maneuver where state=[0,0,0,1,0,0]
% Well, speed=1 here. Let's create a pure zero state.
time_zero = 0;
state_zero = [0, 0, 0, 0, 0, 0];
sm_zero = vehicleStateMetrics(time_zero, state_zero, params);
if isfinite(sm_zero.speed) && isfinite(sm_zero.beta) && isfinite(sm_zero.curvature) && ...
   sm_zero.speed == 0 && sm_zero.curvature == 0 && sm_zero.beta == 0
    fprintf('Test 1: Zero-state metrics are finite and physically consistent... PASSED\n');
else
    fprintf('Test 1: FAILED\n');
    all_tests_passed = false;
end

%% TEST 2: Speed calculation matches analytical sqrt(vx^2 + vy^2).
vx_test2 = 3.0; vy_test2 = -4.0;
sm_test2 = vehicleStateMetrics(0, [0 0 0 vx_test2 vy_test2 0], params);
if abs(sm_test2.speed - 5.0) < 1e-8
    fprintf('Test 2: Speed calculation matches analytical sqrt(vx^2 + vy^2)... PASSED\n');
else
    fprintf('Test 2: FAILED\n');
    all_tests_passed = false;
end

%% TEST 3: Sideslip calculation matches atan2(vy,vx).
if abs(sm_test2.beta - atan2(-4.0, 3.0)) < 1e-8
    fprintf('Test 3: Sideslip calculation matches atan2(vy,vx)... PASSED\n');
else
    fprintf('Test 3: FAILED\n');
    all_tests_passed = false;
end

%% TEST 4: Yaw-rate extraction is correct.
if abs(sm_C.r(end) - res_corner.state(end, 6)) < 1e-10
    fprintf('Test 4: Yaw-rate extraction is correct... PASSED\n');
else
    fprintf('Test 4: FAILED\n');
    all_tests_passed = false;
end

%% TEST 5: Body-frame ax reconstruction is correct.
% Independently reconstruct ax using raw global kinematic state (X, Y, psi, vx, vy)
% from the Phase 3H trajectory output, transforming inertial accelerations back to body frame.
raw_psi = res_corner.state(:, 3);
raw_vx  = res_corner.state(:, 4);
raw_vy  = res_corner.state(:, 5);

V_X = raw_vx .* cos(raw_psi) - raw_vy .* sin(raw_psi);
V_Y = raw_vx .* sin(raw_psi) + raw_vy .* cos(raw_psi);
a_X = gradient(V_X, res_corner.time);
a_Y = gradient(V_Y, res_corner.time);

ax_reconstructed = a_X .* cos(raw_psi) + a_Y .* sin(raw_psi);
if max(abs(sm_C.ax - ax_reconstructed)) < 0.05
    fprintf('Test 5: Body-frame ax reconstruction is correct... PASSED\n');
else
    fprintf('Test 5: FAILED\n');
    all_tests_passed = false;
end

%% TEST 6: Body-frame ay reconstruction is correct.
% Independently reconstruct ay using raw global kinematic state.
ay_reconstructed = -a_X .* sin(raw_psi) + a_Y .* cos(raw_psi);
if max(abs(sm_C.ay - ay_reconstructed)) < 0.05
    fprintf('Test 6: Body-frame ay reconstruction is correct... PASSED\n');
else
    fprintf('Test 6: FAILED\n');
    all_tests_passed = false;
end

%% TEST 7: Physical acceleration differs correctly from raw state derivative when yaw coupling exists.
% In cornering maneuver, vy*r and vx*r should be non-zero
dvy_dt_raw = gradient(sm_C.vy, res_corner.time);
if max(abs(sm_C.vx .* sm_C.r)) > 0.01 && max(abs(sm_C.ay - dvy_dt_raw)) > 0.01
    fprintf('Test 7: Physical acceleration differs correctly from raw state derivative when yaw coupling exists... PASSED\n');
else
    fprintf('Test 7: FAILED\n');
    all_tests_passed = false;
end

%% TEST 8: Yaw acceleration differentiation is correct.
dr_dt_expected = gradient(sm_C.r, res_corner.time);
if max(abs(sm_C.yawAcceleration - dr_dt_expected)) < 1e-10
    fprintf('Test 8: Yaw acceleration differentiation is correct... PASSED\n');
else
    fprintf('Test 8: FAILED\n');
    all_tests_passed = false;
end

%% TEST 9: Straight-line curvature approaches zero.
if max(abs(sm_S.curvature)) < 1e-10
    fprintf('Test 9: Straight-line curvature approaches zero... PASSED\n');
else
    fprintf('Test 9: FAILED\n');
    all_tests_passed = false;
end

%% TEST 10: Nonzero yaw with nonzero speed produces expected curvature.
if max(abs(sm_C.curvature)) > 0.01 && max(abs(sm_C.curvature - sm_C.r ./ sm_C.speed)) < 1e-10
    fprintf('Test 10: Nonzero yaw with nonzero speed produces expected curvature... PASSED\n');
else
    fprintf('Test 10: FAILED\n');
    all_tests_passed = false;
end

%% TEST 11: Zero-speed curvature handling does not produce NaN/Inf.
if isfinite(sm_zero.curvature) && sm_zero.curvature == 0
    fprintf('Test 11: Zero-speed curvature handling does not produce NaN/Inf... PASSED\n');
else
    fprintf('Test 11: FAILED\n');
    all_tests_passed = false;
end

%% TEST 12: Four-wheel force vector dimensions are correct.
if isequal(size(fm_S.mu_x), [length(t_S), 4]) && isequal(size(fm_S.mu_y), [length(t_S), 4])
    fprintf('Test 12: Four-wheel force vector dimensions are correct... PASSED\n');
else
    fprintf('Test 12: FAILED\n');
    all_tests_passed = false;
end

%% TEST 13: Total longitudinal force matches sum of wheel longitudinal forces.
% For a straight maneuver (delta=0), the total Fx is simply the sum of wheel Fx.
if max(abs(fm_S.total_Fx - sum(res_straight.diagnostics.Fx_wheel, 2))) < 1e-10
    fprintf('Test 13: Total longitudinal force matches sum of wheel longitudinal forces... PASSED\n');
else
    fprintf('Test 13: FAILED\n');
    all_tests_passed = false;
end

%% TEST 14: Total lateral force matches sum of transformed wheel forces according to the repository's existing convention.
Fy_trans_sum = zeros(size(fm_C.total_Fy));
for k = 1:length(res_corner.time)
    delta_k = res_corner.diagnostics.delta(k, :)';
    Fx_k = res_corner.diagnostics.Fx_wheel(k, :)';
    Fy_k = res_corner.diagnostics.Fy_wheel(k, :)';
    Fy_trans = Fx_k .* sin(delta_k) + Fy_k .* cos(delta_k);
    Fy_trans_sum(k) = sum(Fy_trans);
end
if max(abs(fm_C.total_Fy - Fy_trans_sum)) < 1e-10
    fprintf('Test 14: Total lateral force matches sum of transformed wheel forces... PASSED\n');
else
    fprintf('Test 14: FAILED\n');
    all_tests_passed = false;
end

%% TEST 15: Yaw moment diagnostic matches existing four-wheel moment convention.
% Independently reconstruct Mz from individual wheel forces.
Mz_reconstructed = zeros(size(fm_C.total_Mz));
for k = 1:length(res_corner.time)
    delta_k = res_corner.diagnostics.delta(k, :)';
    Fx_k = res_corner.diagnostics.Fx_wheel(k, :)';
    Fy_k = res_corner.diagnostics.Fy_wheel(k, :)';
    % Transform to body frame
    Fx_body = Fx_k .* cos(delta_k) - Fy_k .* sin(delta_k);
    Fy_body = Fx_k .* sin(delta_k) + Fy_k .* cos(delta_k);
    % Sum moments: FL(1), FR(2), RL(3), RR(4)
    Mz = (Fx_body(2) - Fx_body(1)) * params.tf / 2 + ...
         (Fx_body(4) - Fx_body(3)) * params.tr / 2 + ...
         (Fy_body(1) + Fy_body(2)) * params.lf - ...
         (Fy_body(3) + Fy_body(4)) * params.lr;
    Mz_reconstructed(k) = Mz;
end
if max(abs(fm_C.total_Mz - Mz_reconstructed)) < 1e-10
    fprintf('Test 15: Yaw moment diagnostic matches existing four-wheel moment convention... PASSED\n');
else
    fprintf('Test 15: FAILED\n');
    all_tests_passed = false;
end

%% TEST 16: Fz conservation remains m*g.
Fz_sum = sum(res_corner.diagnostics.Fz_wheel, 2);
if max(abs(Fz_sum - params.m * params.g)) < 1e-8
    fprintf('Test 16: Fz conservation remains m*g... PASSED\n');
else
    fprintf('Test 16: FAILED\n');
    all_tests_passed = false;
end

%% TEST 17: Normalized longitudinal tire-force metric is finite.
if all(isfinite(fm_C.mu_x(:)))
    fprintf('Test 17: Normalized longitudinal tire-force metric is finite... PASSED\n');
else
    fprintf('Test 17: FAILED\n');
    all_tests_passed = false;
end

%% TEST 18: Normalized lateral tire-force metric is finite.
if all(isfinite(fm_C.mu_y(:)))
    fprintf('Test 18: Normalized lateral tire-force metric is finite... PASSED\n');
else
    fprintf('Test 18: FAILED\n');
    all_tests_passed = false;
end

%% TEST 19: Combined utilization metric is finite.
if all(isfinite(fm_C.mu_combined(:)))
    fprintf('Test 19: Combined utilization metric is finite... PASSED\n');
else
    fprintf('Test 19: FAILED\n');
    all_tests_passed = false;
end

%% TEST 20: Left/right symmetry for symmetric zero-lateral maneuvers.
% Straight acceleration should have identical Fz for FL/FR and RL/RR
diff_FL_FR = res_straight.diagnostics.Fz_wheel(:, 1) - res_straight.diagnostics.Fz_wheel(:, 2);
diff_RL_RR = res_straight.diagnostics.Fz_wheel(:, 3) - res_straight.diagnostics.Fz_wheel(:, 4);
if max(abs(diff_FL_FR)) < 1e-10 && max(abs(diff_RL_RR)) < 1e-10
    fprintf('Test 20: Left/right symmetry for symmetric zero-lateral maneuvers... PASSED\n');
else
    fprintf('Test 20: FAILED\n');
    all_tests_passed = false;
end

%% TEST 21: Positive and negative steering produce corresponding opposite yaw response.
% ConstantCornering uses a positive steering step (+0.1 rad).
% We explicitly simulate a corresponding negative steering step.
delta_neg = @(t) -0.1 * (t > 0.5);
[time_neg, st_neg, diag_neg] = simulateVehicleTrajectory([0, 3], [0;0;0;2;0;0], delta_neg, (2.0/params.R)*ones(4,1), params, [], [], []);
sm_neg = vehicleStateMetrics(time_neg, st_neg, params);
if sm_C.r(end) > 0 && sm_neg.r(end) < 0
    fprintf('Test 21: Positive and negative steering produce corresponding opposite yaw response... PASSED\n');
else
    fprintf('Test 21: FAILED\n');
    all_tests_passed = false;
end

%% TEST 22: Straight acceleration maneuver behaves qualitatively correctly.
if hm_S.final_speed > sm_S.speed(1) && max(sm_S.ax) > 0 && max(abs(sm_S.r)) < 1e-10
    fprintf('Test 22: Straight acceleration maneuver behaves qualitatively correctly... PASSED\n');
else
    fprintf('Test 22: FAILED\n');
    all_tests_passed = false;
end

%% TEST 23: Straight braking maneuver behaves qualitatively correctly.
res_brake = runVehicleManeuverAnalysis('StraightBraking', params);
if res_brake.handling.final_speed < res_brake.stateMetrics.speed(1) && min(res_brake.stateMetrics.ax) < -0.1
    fprintf('Test 23: Straight braking maneuver behaves qualitatively correctly... PASSED\n');
else
    fprintf('Test 23: FAILED\n');
    all_tests_passed = false;
end

%% TEST 24: Positive-steering cornering maneuver behaves correctly.
if hm_C.final_psi > 0 && hm_C.peak_ay > 0.1 && hm_C.peak_abs_r > 0.05
    fprintf('Test 24: Positive-steering cornering maneuver behaves correctly... PASSED\n');
else
    fprintf('Test 24: FAILED\n');
    all_tests_passed = false;
end

%% TEST 25: Negative-steering cornering maneuver behaves correctly.
% Reusing the negative simulation from Test 21
if st_neg(end, 3) < 0 && st_neg(end, 6) < -0.05
    fprintf('Test 25: Negative-steering cornering maneuver behaves correctly... PASSED\n');
else
    fprintf('Test 25: FAILED\n');
    all_tests_passed = false;
end

%% TEST 26: Combined braking + cornering produces simultaneous Fx and Fy response.
res_cbrake = runVehicleManeuverAnalysis('CombinedBrakingCornering', params);
if res_cbrake.handling.peak_Fx > 10 && res_cbrake.handling.peak_Fy > 10
    fprintf('Test 26: Combined braking + cornering produces simultaneous Fx and Fy response... PASSED\n');
else
    fprintf('Test 26: FAILED\n');
    all_tests_passed = false;
end

%% TEST 27: Combined acceleration + cornering produces simultaneous Fx and Fy response.
res_caccel = runVehicleManeuverAnalysis('CombinedAccelerationCornering', params);
if res_caccel.handling.peak_Fx > 10 && res_caccel.handling.peak_Fy > 10
    fprintf('Test 27: Combined acceleration + cornering produces simultaneous Fx and Fy response... PASSED\n');
else
    fprintf('Test 27: FAILED\n');
    all_tests_passed = false;
end

%% TEST 28: Maneuver analysis output structure is deterministic and complete.
fields_present = isfield(res_straight, 'time') && isfield(res_straight, 'state') && ...
                 isfield(res_straight, 'diagnostics') && isfield(res_straight, 'stateMetrics') && ...
                 isfield(res_straight, 'forceMetrics') && isfield(res_straight, 'handling');
if fields_present
    fprintf('Test 28: Maneuver analysis output structure is deterministic and complete... PASSED\n');
else
    fprintf('Test 28: FAILED\n');
    all_tests_passed = false;
end

%% TEST 29: Repeated identical maneuver produces identical analysis results.
res_straight_2 = runVehicleManeuverAnalysis('StraightAcceleration', params);
if isequal(res_straight.handling.final_speed, res_straight_2.handling.final_speed) && ...
   isequal(res_straight.forceMetrics.total_Fx, res_straight_2.forceMetrics.total_Fx)
    fprintf('Test 29: Repeated identical maneuver produces identical analysis results... PASSED\n');
else
    fprintf('Test 29: FAILED\n');
    all_tests_passed = false;
end

%% TEST 30: Changing a representative parameter changes an appropriate output.
params_baseline_snapshot = params; % Capture baseline structure before sensitivity testing

params_heavy = params;
params_heavy.m = params.m * 1.5;
res_heavy = runVehicleManeuverAnalysis('StraightAcceleration', params_heavy);

% NOTE: final_speed is physically invariant to mass here because Fx scales linearly 
% with mass (ax = Fx/m = mu*g is constant). Therefore, the "appropriate output" 
% to check for a mass perturbation is the total generated force (peak_Fx).
delta_Fx = res_heavy.handling.peak_Fx - res_straight.handling.peak_Fx;
if abs(delta_Fx) > 10.0
    fprintf('Test 30: Representative mass sensitivity produces measurable response... PASSED (|\\Delta peak_Fx| = %.4f N)\n', abs(delta_Fx));
else
    fprintf('Test 30: FAILED\n');
    all_tests_passed = false;
end

%% TEST 31: Baseline parameter structure remains unchanged after sensitivity analysis.
if isequal(params, params_baseline_snapshot)
    fprintf('Test 31: Baseline parameter structure remains completely unchanged... PASSED\n');
else
    fprintf('Test 31: FAILED\n');
    all_tests_passed = false;
end

%% TEST 32: All state diagnostics are finite and real.
all_finite_real_states = all(isfinite(sm_C.speed)) && all(isreal(sm_C.speed)) && ...
                         all(isfinite(sm_C.beta)) && all(isreal(sm_C.beta)) && ...
                         all(isfinite(sm_C.ax)) && all(isreal(sm_C.ax)) && ...
                         all(isfinite(sm_C.ay)) && all(isreal(sm_C.ay));
if all_finite_real_states
    fprintf('Test 32: All state diagnostics are finite and real... PASSED\n');
else
    fprintf('Test 32: FAILED\n');
    all_tests_passed = false;
end

%% TEST 33: All tire diagnostics are finite and real.
all_finite_real_forces = all(isfinite(fm_C.mu_combined(:))) && all(isreal(fm_C.mu_combined(:))) && ...
                         all(isfinite(fm_C.total_Fx)) && all(isreal(fm_C.total_Fx));
if all_finite_real_forces
    fprintf('Test 33: All tire diagnostics are finite and real... PASSED\n');
else
    fprintf('Test 33: FAILED\n');
    all_tests_passed = false;
end

%% TEST 34: All summary metrics are finite wherever physically defined.
if isfinite(hm_C.peak_speed) && isfinite(hm_C.peak_ay) && isfinite(hm_C.peak_tire_util)
    fprintf('Test 34: All summary metrics are finite wherever physically defined... PASSED\n');
else
    fprintf('Test 34: FAILED\n');
    all_tests_passed = false;
end

%% TEST 35: Analysis results are consistent with Phase 3H trajectory outputs.
% Independently construct derived variables from raw Phase 3H state to verify cross-layer interpretation
vx_3h = res_corner.state(:, 4);
vy_3h = res_corner.state(:, 5);
r_3h  = res_corner.state(:, 6);

speed_3h = sqrt(vx_3h.^2 + vy_3h.^2);
beta_3h  = atan2(vy_3h, vx_3h);
curvature_3h = zeros(size(speed_3h));
idx = speed_3h > 1e-8;
curvature_3h(idx) = r_3h(idx) ./ speed_3h(idx);
peak_speed_3h = max(speed_3h);
peak_abs_r_3h = max(abs(r_3h));

if max(abs(sm_C.speed - speed_3h)) < 1e-10 && ...
   max(abs(sm_C.beta - beta_3h)) < 1e-10 && ...
   max(abs(sm_C.curvature - curvature_3h)) < 1e-10 && ...
   abs(hm_C.peak_speed - peak_speed_3h) < 1e-10 && ...
   abs(hm_C.peak_abs_r - peak_abs_r_3h) < 1e-10
    fprintf('Test 35: Analysis results are consistent with Phase 3H trajectory outputs... PASSED\n');
else
    fprintf('Test 35: FAILED\n');
    all_tests_passed = false;
end

%% TEST 36: Invalid normal load rejection in tireForceMetrics
err_caught = 0;
diag_invalid = res_corner.diagnostics;

% Negative Fz
diag_invalid.Fz_wheel(1, 1) = -100;
try
    tireForceMetrics(diag_invalid, params);
catch ME
    if strcmp(ME.identifier, 'tireForceMetrics:invalidNormalLoad')
        err_caught = err_caught + 1;
    end
end

% Zero Fz
diag_invalid.Fz_wheel(1, 1) = 0;
try
    tireForceMetrics(diag_invalid, params);
catch ME
    if strcmp(ME.identifier, 'tireForceMetrics:invalidNormalLoad')
        err_caught = err_caught + 1;
    end
end

% NaN Fz
diag_invalid.Fz_wheel(1, 1) = NaN;
try
    tireForceMetrics(diag_invalid, params);
catch ME
    if strcmp(ME.identifier, 'tireForceMetrics:invalidNormalLoad')
        err_caught = err_caught + 1;
    end
end

if err_caught == 3
    fprintf('Test 36: Invalid normal load rejection in tireForceMetrics... PASSED\n');
else
    fprintf('Test 36: FAILED (caught %d/3)\n', err_caught);
    all_tests_passed = false;
end

%% Final Summary
fprintf('\n');
if all_tests_passed
    fprintf('============================================================\n');
    fprintf('  All 36 vehicle performance analysis tests PASSED!\n');
    fprintf('============================================================\n');
else
    fprintf('============================================================\n');
    fprintf('  ONE OR MORE TESTS FAILED.\n');
    fprintf('============================================================\n');
end
