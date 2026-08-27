% verifyVehicleSimulation.m
% Verification script for Phase 3G: Time-Domain Vehicle Simulation Framework (Fixed-Step RK4)

clc;
clear;
close all;

fprintf('============================================================\n');
fprintf('  Running Verification: Time-Domain Vehicle Simulation (3G)\n');
fprintf('============================================================\n\n');

params = roverParameters();
R = params.R; % 0.15 m
m = params.m; % 60.0 kg
Iz = params.Iz; % 5.0 kg*m^2
all_tests_passed = true;

%% Test 1: State dimension correctness
x0_1 = [0; 0; 0; 2.0; 0; 0];
st1 = @(t) 0.0;
ws1 = @(t) (2.0 / R) * ones(4, 1);
tFinal1 = 1.0;
dt1 = 0.01;

[sHist1, t1] = runVehicleSimulation(x0_1, st1, ws1, tFinal1, dt1, params);

if size(sHist1, 2) == 6 && size(sHist1, 1) == length(t1)
    fprintf('Test 1: State dimension correctness (N x 6 matrix)... PASSED\n');
else
    fprintf('Test 1: FAILED\n');
    all_tests_passed = false;
end

%% Test 2: Time vector correctness
t_expected = (0:dt1:tFinal1)';
if length(t1) == length(t_expected) && max(abs(t1 - t_expected)) < 1e-12 && t1(1) == 0 && abs(t1(end) - tFinal1) < 1e-12
    fprintf('Test 2: Time vector correctness (starts at 0, ends at tFinal, uniform dt)... PASSED\n');
else
    fprintf('Test 2: FAILED\n');
    all_tests_passed = false;
end

%% Test 3: RK4 execution
if ~isempty(sHist1) && ~isempty(t1) && length(t1) > 1
    fprintf('Test 3: RK4 execution (completes without exception)... PASSED\n');
else
    fprintf('Test 3: FAILED\n');
    all_tests_passed = false;
end

%% Test 4: Zero-input equilibrium (all states zero -> remain zero)
x0_zero = zeros(6, 1);
[sHist4, ~] = runVehicleSimulation(x0_zero, 0.0, zeros(4, 1), 1.0, 0.01, params);

if max(max(abs(sHist4))) < 1e-10
    fprintf('Test 4: Zero-input equilibrium (all states remain identically zero)... PASSED\n');
else
    fprintf('Test 4: FAILED\n');
    all_tests_passed = false;
end

%% Test 5: Straight-line motion (Y=0, psi=0, r=0, X=vx*t)
if max(abs(sHist1(:, 2))) < 1e-8 && ...
   max(abs(sHist1(:, 3))) < 1e-8 && ...
   max(abs(sHist1(:, 5))) < 1e-8 && ...
   max(abs(sHist1(:, 6))) < 1e-8 && ...
   abs(sHist1(end, 1) - 2.0 * tFinal1) < 1e-4
    fprintf('Test 5: Straight-line motion (pure rolling -> Y=0, psi=0, X=vx*t)... PASSED\n');
else
    fprintf('Test 5: FAILED\n');
    all_tests_passed = false;
end

%% Test 6: Constant heading preservation
psi0_6 = 0.5;
x0_6 = [0; 0; psi0_6; 2.0; 0; 0];
[sHist6, ~] = runVehicleSimulation(x0_6, 0.0, ws1, 1.0, 0.01, params);

if max(abs(sHist6(:, 3) - psi0_6)) < 1e-8 && max(abs(sHist6(:, 6))) < 1e-8
    fprintf('Test 6: Constant heading preservation (psi remains exactly psi0)... PASSED\n');
else
    fprintf('Test 6: FAILED\n');
    all_tests_passed = false;
end

%% Test 7: Positive steering produces positive yaw (delta > 0 -> r > 0, psi > 0, Y > 0)
st7 = 0.05;
ws7 = [(2.0 * cos(st7) / R); (2.0 * cos(st7) / R); (2.0 / R); (2.0 / R)];
[sHist7, ~] = runVehicleSimulation(x0_1, st7, ws7, 1.0, 0.01, params);

if sHist7(end, 6) > 0 && sHist7(end, 3) > 0 && sHist7(end, 2) > 0
    fprintf('Test 7: Positive steering produces positive yaw (delta > 0 -> r > 0, psi > 0, Y > 0)... PASSED\n');
else
    fprintf('Test 7: FAILED\n');
    all_tests_passed = false;
end

%% Test 8: Negative steering produces negative yaw (delta < 0 -> r < 0, psi < 0, Y < 0)
st8 = -0.05;
ws8 = [(2.0 * cos(st8) / R); (2.0 * cos(st8) / R); (2.0 / R); (2.0 / R)];
[sHist8, ~] = runVehicleSimulation(x0_1, st8, ws8, 1.0, 0.01, params);

if sHist8(end, 6) < 0 && sHist8(end, 3) < 0 && sHist8(end, 2) < 0
    fprintf('Test 8: Negative steering produces negative yaw (delta < 0 -> r < 0, psi < 0, Y < 0)... PASSED\n');
else
    fprintf('Test 8: FAILED\n');
    all_tests_passed = false;
end

%% Test 9: Position integration correctness
% Verify displacement along rotated heading psi = pi/6
psi0_9 = pi / 6.0;
x0_9 = [10.0; -5.0; psi0_9; 2.0; 0.0; 0.0];
[sHist9, t9] = runVehicleSimulation(x0_9, 0.0, ws1, 1.0, 0.01, params);

expected_X9 = 10.0 + 2.0 * cos(psi0_9) * t9;
expected_Y9 = -5.0 + 2.0 * sin(psi0_9) * t9;

if max(abs(sHist9(:, 1) - expected_X9)) < 1e-4 && max(abs(sHist9(:, 2) - expected_Y9)) < 1e-4
    fprintf('Test 9: Position integration correctness (matches analytical global ray)... PASSED\n');
else
    fprintf('Test 9: FAILED\n');
    all_tests_passed = false;
end

%% Test 10: Constant velocity analytical comparison
err_X10 = max(abs(sHist1(:, 1) - 2.0 * t1));
err_vx10 = max(abs(sHist1(:, 4) - 2.0));

if err_X10 < 1e-4 && err_vx10 < 1e-8
    fprintf('Test 10: Constant velocity analytical comparison (error < 1e-4 m)... PASSED\n');
else
    fprintf('Test 10: FAILED\n');
    all_tests_passed = false;
end

%% Test 11: Simulation reproducibility
[sHist11_a, t11_a] = runVehicleSimulation(x0_1, st7, ws7, 1.0, 0.01, params);
[sHist11_b, t11_b] = runVehicleSimulation(x0_1, st7, ws7, 1.0, 0.01, params);

if isequal(sHist11_a, sHist11_b) && isequal(t11_a, t11_b)
    fprintf('Test 11: Simulation reproducibility (exact bitwise match across identical runs)... PASSED\n');
else
    fprintf('Test 11: FAILED\n');
    all_tests_passed = false;
end

%% Test 12: No NaN (check all major trajectories produced up to this point)
has_nan = any(isnan(sHist1(:))) || any(isnan(sHist4(:))) || any(isnan(sHist6(:))) || ...
          any(isnan(sHist7(:))) || any(isnan(sHist8(:))) || any(isnan(sHist9(:)));
if ~has_nan
    fprintf('Test 12: No NaN (sHist1/4/6/7/8/9 — all integrated state elements free of NaN)... PASSED\n');
else
    fprintf('Test 12: FAILED\n');
    all_tests_passed = false;
end

%% Test 13: No Inf (check all major trajectories produced up to this point)
has_inf = any(isinf(sHist1(:))) || any(isinf(sHist4(:))) || any(isinf(sHist6(:))) || ...
          any(isinf(sHist7(:))) || any(isinf(sHist8(:))) || any(isinf(sHist9(:)));
if ~has_inf
    fprintf('Test 13: No Inf (sHist1/4/6/7/8/9 — all state trajectories bounded and free of Inf)... PASSED\n');
else
    fprintf('Test 13: FAILED\n');
    all_tests_passed = false;
end

%% Test 14: Variable dt consistency
[sHist14_coarse, ~] = runVehicleSimulation(x0_1, st7, ws7, 1.0, 0.01, params);
[sHist14_fine, ~]   = runVehicleSimulation(x0_1, st7, ws7, 1.0, 0.005, params);

diff14 = max(abs(sHist14_coarse(end, :) - sHist14_fine(end, :)));
if diff14 < 1e-3
    fprintf('Test 14: Variable dt consistency (dt=0.01 vs dt=0.005 endpoint diff = %.2e < 1e-3)... PASSED\n', diff14);
else
    fprintf('Test 14: FAILED\n');
    all_tests_passed = false;
end

%% Test 15: Small dt convergence
[sHist15_dt1, ~] = runVehicleSimulation(x0_1, st7, ws7, 0.5, 0.02, params);
[sHist15_dt2, ~] = runVehicleSimulation(x0_1, st7, ws7, 0.5, 0.01, params);
[sHist15_dt4, ~] = runVehicleSimulation(x0_1, st7, ws7, 0.5, 0.005, params);

err1 = norm(sHist15_dt1(end, :) - sHist15_dt2(end, :));
err2 = norm(sHist15_dt2(end, :) - sHist15_dt4(end, :));

if err2 < err1
    fprintf('Test 15: Small dt convergence (RK4 error decreases with smaller dt: %.2e -> %.2e)... PASSED\n', err1, err2);
else
    fprintf('Test 15: FAILED\n');
    all_tests_passed = false;
end

%% Test 16: Wheel speed callback execution
global ws_eval_times;
ws_eval_times = [];

[sHist16, t16] = runVehicleSimulation(x0_1, 0.0, @(t) recordAndReturnWS(t, R), 0.5, 0.01, params);

has_evaluations = ~isempty(ws_eval_times);
if has_evaluations
    unique_non_zero_times = unique(ws_eval_times(ws_eval_times > 0));
    has_time_dependence = length(unique_non_zero_times) > 5;
else
    has_time_dependence = false;
end

if has_evaluations && has_time_dependence && sHist16(end, 4) > x0_1(4)
    fprintf('Test 16: Wheel speed callback execution (callback exercised dynamically over time)... PASSED\n');
    fprintf('         Evaluated %d times, unique times tracked: %d\n', length(ws_eval_times), length(unique_non_zero_times));
else
    fprintf('Test 16: FAILED (evaluations: %d, time-dependent: %d)\n', has_evaluations, has_time_dependence);
    all_tests_passed = false;
end
clear global ws_eval_times;

%% Test 17: Steering callback execution
st17_fcn = @(t) 0.05 * sin(2 * pi * t);
[sHist17, ~] = runVehicleSimulation(x0_1, st17_fcn, ws1, 1.0, 0.01, params);

if max(abs(sHist17(:, 3))) > 0.01 && ~any(isnan(sHist17(:)))
    fprintf('Test 17: Steering callback execution (sinusoidal steering smoothly integrated)... PASSED\n');
else
    fprintf('Test 17: FAILED\n');
    all_tests_passed = false;
end

%% Test 18: State-history dimensions
x0_diff = [1; 2; 3; 1.5; 0.1; 0.05];
[sHist18, t18] = runVehicleSimulation(x0_diff, 0.01, ws1, 0.4, 0.02, params);

if isequal(size(sHist18), [length(t18), 6])
    fprintf('Test 18: State-history dimensions (arbitrary initial state, exact N x 6 sizing)... PASSED\n');
else
    fprintf('Test 18: FAILED\n');
    all_tests_passed = false;
end

%% Test 19: Longitudinal acceleration response (omega > vx/R -> dot_vx > 0, vx increases)
ws19 = (2.5 / R) * ones(4, 1); % Driving torque/slip
[sHist19, ~] = runVehicleSimulation(x0_1, 0.0, ws19, 0.5, 0.01, params);

if sHist19(end, 4) > x0_1(4) && sHist19(end, 1) > 1.0
    fprintf('Test 19: Longitudinal acceleration response (positive slip increases forward velocity)... PASSED\n');
else
    fprintf('Test 19: FAILED\n');
    all_tests_passed = false;
end

%% Test 20: Braking response (omega < vx/R -> dot_vx < 0, vx decreases)
ws20 = (1.5 / R) * ones(4, 1); % Braking slip
[sHist20, ~] = runVehicleSimulation(x0_1, 0.0, ws20, 0.5, 0.01, params);

if sHist20(end, 4) < x0_1(4) && sHist20(end, 4) > 0
    fprintf('Test 20: Braking response (negative slip decelerates vehicle forward velocity)... PASSED\n');
else
    fprintf('Test 20: FAILED\n');
    all_tests_passed = false;
end

%% Test 21: Cornering response (steady steering establishes positive yaw rate and lateral arc)
[sHist21, t21] = runVehicleSimulation(x0_1, 0.04, ws1, 1.0, 0.01, params);

if sHist21(end, 6) > 0 && sHist21(end, 3) > 0.05 && sHist21(end, 2) > 0.01
    fprintf('Test 21: Cornering response (steady steering establishes positive yaw rate and lateral arc)... PASSED\n');
else
    fprintf('Test 21: FAILED\n');
    all_tests_passed = false;
end

%% Test 22: Combined cornering and acceleration
st22 = 0.04;
ws22 = (2.4 / R) * ones(4, 1); % Cornering + driving
[sHist22, ~] = runVehicleSimulation(x0_1, st22, ws22, 0.5, 0.01, params);

if sHist22(end, 4) > x0_1(4) && sHist22(end, 6) > 0 && sHist22(end, 2) > 0
    fprintf('Test 22: Combined cornering and acceleration (simultaneous speed increase and yaw turn)... PASSED\n');
else
    fprintf('Test 22: FAILED\n');
    all_tests_passed = false;
end

%% Test 23: 6-state ODE/RK4 integration consistency
% Verifies that runVehicleSimulation's RK4 ODE evaluation is consistent
% with an independently reconstructed one-step RK4 using the same Phase 3F
% derivative interface. Detects errors in: state indexing, world/body
% coordinate mapping, RK4 stage times/states, or k1–k4 combination.
sample_state = sHist21(25, :)';
t_s = t21(25);
delta_s = 0.04;

% Independent one-step RK4 reconstruction using the same 6-state ODE
h = 0.01;
ode_independent = @(t, x_val) [ ...
    x_val(4)*cos(x_val(3)) - x_val(5)*sin(x_val(3)); ...
    x_val(4)*sin(x_val(3)) + x_val(5)*cos(x_val(3)); ...
    x_val(6); ...
    closedLoopVehicleDerivative(x_val(4:6), delta_s, ws1(t), params) ...
];

k1 = ode_independent(t_s,           sample_state);
k2 = ode_independent(t_s + 0.5*h,   sample_state + 0.5*h*k1);
k3 = ode_independent(t_s + 0.5*h,   sample_state + 0.5*h*k2);
k4 = ode_independent(t_s + h,       sample_state + h*k3);
x_next_reconstructed = sample_state + (h/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);

% One-step output from the simulator starting from same state
[sHist_step, ~] = runVehicleSimulation(sample_state, delta_s, ws1, h, h, params);
x_next_sim = sHist_step(2, :)';

max_consistency_error = max(abs(x_next_sim - x_next_reconstructed));

if max_consistency_error < 1e-12
    fprintf('Test 23: 6-state ODE/RK4 integration consistency (simulator matches independent reconstruction)... PASSED\n');
    fprintf('         maximum RK4 consistency error = %.2e\n', max_consistency_error);
else
    fprintf('Test 23: FAILED (RK4 consistency error = %.2e)\n', max_consistency_error);
    all_tests_passed = false;
end

%% Test 24: Numerical stability (long duration maneuver remains stable and bounded)
tFinal24 = 3.0;
st24_fcn = @(t) 0.03 * sin(pi * t);
[sHist24, t24] = runVehicleSimulation(x0_1, st24_fcn, ws1, tFinal24, 0.01, params);

if ~any(isnan(sHist24(:))) && ~any(isinf(sHist24(:))) && sHist24(end, 4) > 1.0 && sHist24(end, 4) < 3.0
    fprintf('Test 24: Numerical stability (3-second continuous dynamic maneuver remains stable)... PASSED\n');
else
    fprintf('Test 24: FAILED\n');
    all_tests_passed = false;
end

%% Test 25: Regression compatibility (custom tire and vehicle parameters)
% Verifies that custom pure-slip and combined-slip parameter structs are
% passed through and applied, not silently ignored.
% Custom params use intentionally different values from the built-in defaults
% (default: Bx=10, Cx=1.90, Dx=1.00, Ex=0.97, By=10, Cy=1.30, Dy=1.00, Ey=-1.0)
% so that custom vs. default trajectory comparison is meaningful.
pure_params_custom = struct('Bx', 5.0,  'Cx', 1.50, 'Dx', 0.70, 'Ex', 0.50, ...
                            'By', 6.0,  'Cy', 1.10, 'Dy', 0.75, 'Ey', -0.50);
combined_params_custom = struct('Bx_alpha', 5.0,  'Cx_alpha', 0.80, 'Ex_alpha', -0.50, 'SHx_alpha', 0.0, ...
                                'By_kappa', 6.0,  'Cy_kappa', 0.80, 'Ey_kappa', -0.50, 'SHy_kappa', 0.0);

% Use a non-trivial maneuver so custom tire params actually affect forces
st25 = 0.04;
ws25 = (2.0 / R) * ones(4, 1);

% Run 1: custom parameters
[sHist25a, ~] = runVehicleSimulation(x0_1, st25, ws25, 0.5, 0.01, params, pure_params_custom, combined_params_custom);
% Run 2: identical custom parameters (determinism check)
[sHist25b, ~] = runVehicleSimulation(x0_1, st25, ws25, 0.5, 0.01, params, pure_params_custom, combined_params_custom);
% Run 3: default parameters (verify custom params are not silently ignored)
[sHist25_default, ~] = runVehicleSimulation(x0_1, st25, ws25, 0.5, 0.01, params);

N25 = size(sHist25a, 1);
correct_size       = (size(sHist25a, 2) == 6) && (N25 > 1);
all_finite_real    = all(isfinite(sHist25a(:))) && all(isreal(sHist25a(:)));
state_evolved      = max(abs(sHist25a(end, :) - x0_1')) > 1e-6;
deterministic      = isequal(sHist25a, sHist25b);
params_applied     = max(abs(sHist25a(end, :) - sHist25_default(end, :))) > 1e-10;

if correct_size && all_finite_real && state_evolved && deterministic && params_applied
    fprintf('Test 25: Regression compatibility (custom tire params: size OK, finite/real, evolved, deterministic, differs from default)... PASSED\n');
    fprintf('         Custom vs default endpoint diff = %.4e (params are applied, not ignored)\n', ...
            max(abs(sHist25a(end, :) - sHist25_default(end, :))));
else
    fprintf('Test 25: FAILED (size=%d, finite=%d, evolved=%d, deterministic=%d, params_applied=%d)\n', ...
            correct_size, all_finite_real, state_evolved, deterministic, params_applied);
    all_tests_passed = false;
end

%% Summary
fprintf('\n');
if all_tests_passed
    fprintf('============================================================\n');
    fprintf('  All 25 vehicle simulation tests PASSED!\n');
    fprintf('============================================================\n');
else
    fprintf('============================================================\n');
    fprintf('  ONE OR MORE TESTS FAILED.\n');
    fprintf('============================================================\n');
end

function w = recordAndReturnWS(t, R)
    global ws_eval_times;
    ws_eval_times = [ws_eval_times; t];
    w = ((2.0 + 0.5 * t) / R) * ones(4, 1);
end
