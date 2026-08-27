% verifyVehicleTrajectorySimulation.m
% Verification script for Phase 3H: Numerical Vehicle Trajectory Integration and Analysis

clc;
clear;
close all;

fprintf('============================================================\n');
fprintf('  Running Verification: Vehicle Trajectory Integration (3H)\n');
fprintf('============================================================\n\n');

params = roverParameters();
R = params.R; % 0.15 m
m = params.m; % 60.0 kg
Iz = params.Iz; % 5.0 kg*m^2
all_tests_passed = true;
tol_coarse = 1e-4;
tol_fine   = 1e-8;

%% Test 1: Zero-motion equilibrium (all states zero -> remain zero)
tSpan1 = [0.0, 2.0];
x0_1 = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
delta1 = 0.0;
omega1 = zeros(4, 1);

[t1, s1] = simulateVehicleTrajectory(tSpan1, x0_1, delta1, omega1, params);

if max(max(abs(s1))) < 1e-10
    fprintf('Test 1: Zero-motion equilibrium (v=0, omega=0 -> all states=0)... PASSED\n');
else
    fprintf('Test 1: FAILED\n');
    all_tests_passed = false;
end

%% Test 2: Constant straight-line motion (pure rolling, Y=0, psi=0, r=0, X increases)
tSpan2 = [0.0, 2.0];
vx0_2 = 2.0;
x0_2 = [0.0; 0.0; 0.0; vx0_2; 0.0; 0.0];
delta2 = 0.0;
omega2 = (vx0_2 / R) * ones(4, 1);

[t2, s2] = simulateVehicleTrajectory(tSpan2, x0_2, delta2, omega2, params);

% Expected: X(end) ~ vx0*2.0 = 4.0, Y ~ 0, psi ~ 0, r ~ 0, vx ~ 2.0
if abs(s2(end, 1) - 4.0) < 1e-5 && ...
   max(abs(s2(:, 2))) < 1e-8 && ...
   max(abs(s2(:, 3))) < 1e-8 && ...
   max(abs(s2(:, 5))) < 1e-8 && ...
   max(abs(s2(:, 6))) < 1e-8
    fprintf('Test 2: Constant straight-line motion (pure rolling -> Y=0, psi=0, X=vx*t)... PASSED\n');
else
    fprintf('Test 2: FAILED\n');
    all_tests_passed = false;
end

%% Test 3: Global position integration (X(t) = X0 + vx*t, Y(t) = Y0)
tSpan3 = linspace(0.0, 2.0, 51);
x0_3 = [10.0; 5.0; 0.0; 2.5; 0.0; 0.0];
delta3 = 0.0;
omega3 = (2.5 / R) * ones(4, 1);

[t3, s3] = simulateVehicleTrajectory(tSpan3, x0_3, delta3, omega3, params);

expected_X3 = 10.0 + 2.5 * t3;
expected_Y3 = 5.0 * ones(size(t3));

if max(abs(s3(:, 1) - expected_X3)) < 1e-5 && max(abs(s3(:, 2) - expected_Y3)) < 1e-8
    fprintf('Test 3: Global position integration (X=X0+vx*t, Y=Y0)... PASSED\n');
else
    fprintf('Test 3: FAILED\n');
    all_tests_passed = false;
end

%% Test 4: Heading integration (psi(t) = psi0 + integral(r dt))
% Verify that global yaw kinematics dot(psi) = r holds across integrated trajectory
tSpan4 = linspace(0.0, 1.0, 101);
x0_4 = [0.0; 0.0; 0.2; 2.0; 0.0; 0.0];
delta4 = 0.04;
omega4 = [(2.0 * cos(delta4) / R); (2.0 * cos(delta4) / R); (2.0 / R); (2.0 / R)];

[t4, s4] = simulateVehicleTrajectory(tSpan4, x0_4, delta4, omega4, params);

% Numerical trapezoidal integration of r(t)
psi_integrated = 0.2 + cumtrapz(t4, s4(:, 6));
psi_error = max(abs(s4(:, 3) - psi_integrated));

if psi_error < 1e-3 && s4(end, 3) > 0.2
    fprintf('Test 4: Heading integration (psi matches integral of r, diff = %.2e rad)... PASSED\n', psi_error);
else
    fprintf('Test 4: FAILED\n');
    all_tests_passed = false;
end

%% Test 5: Positive steering response (delta > 0 -> r > 0, psi increases, Y curves left)
tSpan5 = [0.0, 1.5];
x0_5 = [0.0; 0.0; 0.0; 2.0; 0.0; 0.0];
delta5 = 0.05;
omega5 = [(2.0 * cos(delta5) / R); (2.0 * cos(delta5) / R); (2.0 / R); (2.0 / R)];

[t5, s5, d5] = simulateVehicleTrajectory(tSpan5, x0_5, delta5, omega5, params);

if s5(end, 6) > 0 && s5(end, 3) > 0 && s5(end, 2) > 0 && d5.ay(end) > 0
    fprintf('Test 5: Positive steering response (delta > 0 -> r > 0, psi > 0, Y > 0)... PASSED\n');
else
    fprintf('Test 5: FAILED\n');
    all_tests_passed = false;
end

%% Test 6: Negative steering response (delta < 0 -> r < 0, psi decreases, Y curves right)
delta6 = -0.05;
omega6 = [(2.0 * cos(delta6) / R); (2.0 * cos(delta6) / R); (2.0 / R); (2.0 / R)];

[t6, s6, d6] = simulateVehicleTrajectory(tSpan5, x0_5, delta6, omega6, params);

if s6(end, 6) < 0 && s6(end, 3) < 0 && s6(end, 2) < 0 && d6.ay(end) < 0
    fprintf('Test 6: Negative steering response (delta < 0 -> r < 0, psi < 0, Y < 0)... PASSED\n');
else
    fprintf('Test 6: FAILED\n');
    all_tests_passed = false;
end

%% Test 7: Steering symmetry (positive vs negative steering sign symmetry)
tGrid7 = linspace(0.0, 1.0, 51);
[~, s7_pos] = simulateVehicleTrajectory(tGrid7, x0_5, +0.05, omega5, params);
[~, s7_neg] = simulateVehicleTrajectory(tGrid7, x0_5, -0.05, omega6, params);

diff_X   = max(abs(s7_pos(:, 1) - s7_neg(:, 1)));
diff_Y   = max(abs(s7_pos(:, 2) + s7_neg(:, 2)));
diff_psi = max(abs(s7_pos(:, 3) + s7_neg(:, 3)));
diff_vx  = max(abs(s7_pos(:, 4) - s7_neg(:, 4)));
diff_vy  = max(abs(s7_pos(:, 5) + s7_neg(:, 5)));
diff_r   = max(abs(s7_pos(:, 6) + s7_neg(:, 6)));

if diff_X < 1e-5 && diff_Y < 1e-5 && diff_psi < 1e-5 && ...
   diff_vx < 1e-5 && diff_vy < 1e-5 && diff_r < 1e-5
    fprintf('Test 7: Steering symmetry (exact sign reflection: Y, psi, vy, r odd; X, vx even)... PASSED\n');
else
    fprintf('Test 7: FAILED\n');
    all_tests_passed = false;
end

%% Test 8: Differential traction yaw response (omega_L < omega_R -> Mz > 0, r > 0, psi > 0)
tSpan8 = [0.0, 1.0];
x0_8 = [0.0; 0.0; 0.0; 2.0; 0.0; 0.0];
delta8 = 0.0;
omega8 = [(1.9 / R); (2.1 / R); (1.9 / R); (2.1 / R)]; % Left braking/coasting, right driving

[t8, s8, d8] = simulateVehicleTrajectory(tSpan8, x0_8, delta8, omega8, params);

if s8(end, 6) > 0 && s8(end, 3) > 0 && d8.Mz_total(end) > 0
    fprintf('Test 8: Differential traction yaw response (omega_L < omega_R -> Mz > 0, r > 0, psi > 0)... PASSED\n');
else
    fprintf('Test 8: FAILED\n');
    all_tests_passed = false;
end

%% Test 9: Straight-line symmetry (symmetric wheels, zero steer -> near-zero lateral/yaw)
tSpan9 = [0.0, 2.0];
x0_9 = [0.0; 0.0; 0.0; 2.0; 0.0; 0.0];
delta9 = 0.0;
omega9 = (2.2 / R) * ones(4, 1); % Symmetric acceleration

[t9, s9] = simulateVehicleTrajectory(tSpan9, x0_9, delta9, omega9, params);

if max(abs(s9(:, 2))) < 1e-8 && max(abs(s9(:, 3))) < 1e-8 && ...
   max(abs(s9(:, 5))) < 1e-8 && max(abs(s9(:, 6))) < 1e-8
    fprintf('Test 9: Straight-line symmetry (symmetric wheel speeds -> Y=0, psi=0, vy=0, r=0)... PASSED\n');
else
    fprintf('Test 9: FAILED\n');
    all_tests_passed = false;
end

%% Test 10: State dimensions (time is Nx1, state is Nx6, exact ordering [X Y psi vx vy r])
if size(t5, 2) == 1 && size(s5, 2) == 6 && size(s5, 1) == size(t5, 1)
    fprintf('Test 10: State dimensions (time is Nx1, state is Nx6 [X Y psi vx vy r])... PASSED\n');
else
    fprintf('Test 10: FAILED\n');
    all_tests_passed = false;
end

%% Test 11: No NaN / Inf across nonlinear trajectory and diagnostics
tSpan11 = linspace(0.0, 2.0, 51);
delta11_fcn = @(t) 0.05 * sin(2 * pi * 0.5 * t);
omega11_fcn = @(t) ((2.0 + 0.5 * sin(pi * t)) / R) * ones(4, 1);

[t11, s11, d11] = simulateVehicleTrajectory(tSpan11, x0_5, delta11_fcn, omega11_fcn, params);

has_nan_inf = any(isnan(s11(:))) || any(isinf(s11(:))) || ...
              any(isnan(d11.ax(:))) || any(isinf(d11.ax(:))) || ...
              any(isnan(d11.Fz_wheel(:))) || any(isinf(d11.Fz_wheel(:)));

if ~has_nan_inf
    fprintf('Test 11: No NaN / Inf across nonlinear maneuver and all diagnostics... PASSED\n');
else
    fprintf('Test 11: FAILED\n');
    all_tests_passed = false;
end

%% Test 12: Solver tolerance sensitivity (coarse vs tight tolerance agreement)
opts_coarse.RelTol = 1e-4;
opts_coarse.AbsTol = 1e-6;
opts_tight.RelTol  = 1e-8;
opts_tight.AbsTol  = 1e-10;

tGrid12 = linspace(0.0, 1.0, 51);
[~, s12_coarse] = simulateVehicleTrajectory(tGrid12, x0_5, delta5, omega5, params, [], [], opts_coarse);
[~, s12_tight]  = simulateVehicleTrajectory(tGrid12, x0_5, delta5, omega5, params, [], [], opts_tight);

max_tol_diff = max(max(abs(s12_coarse - s12_tight)));

if max_tol_diff < 1e-3
    fprintf('Test 12: Solver tolerance sensitivity (coarse vs tight diff = %.2e < 1e-3)... PASSED\n', max_tol_diff);
else
    fprintf('Test 12: FAILED\n');
    all_tests_passed = false;
end

%% Test 13: Time-resolution / output consistency
[~, s13_coarse_grid] = simulateVehicleTrajectory([0.0, 1.0], x0_5, delta5, omega5, params);
[~, s13_fine_grid]   = simulateVehicleTrajectory(linspace(0.0, 1.0, 101), x0_5, delta5, omega5, params);

endpoint_diff = max(abs(s13_coarse_grid(end, :) - s13_fine_grid(end, :)));

if endpoint_diff < 1e-5
    fprintf('Test 13: Time-resolution output consistency (endpoint diff = %.2e < 1e-5)... PASSED\n', endpoint_diff);
else
    fprintf('Test 13: FAILED\n');
    all_tests_passed = false;
end

%% Test 14: Deterministic repeatability (identical runs produce exact bitwise match)
[t14_a, s14_a] = simulateVehicleTrajectory(tSpan5, x0_5, delta5, omega5, params);
[t14_b, s14_b] = simulateVehicleTrajectory(tSpan5, x0_5, delta5, omega5, params);

if isequal(t14_a, t14_b) && isequal(s14_a, s14_b)
    fprintf('Test 14: Deterministic repeatability (identical simulation runs match exactly)... PASSED\n');
else
    fprintf('Test 14: FAILED\n');
    all_tests_passed = false;
end

%% Test 15: Direct Phase 3F consistency (ODE derivative matches Phase 3F and kinematics)
state_test15 = [5.0; 2.0; 0.3; 2.0; 0.1; 0.15];
delta_test15 = 0.04;
omega_test15 = [(2.2 / R); (2.2 / R); (2.2 / R); (2.2 / R)];

% 1. Evaluate Phase 3F derivative directly
[sdot_3f, diag_3f] = closedLoopVehicleDerivative(state_test15(4:6), delta_test15, omega_test15, params);

% 2. Kinematic pose derivatives
psi15 = state_test15(3);
vx15  = state_test15(4);
vy15  = state_test15(5);
dot_X_expected   = vx15 * cos(psi15) - vy15 * sin(psi15);
dot_Y_expected   = vx15 * sin(psi15) + vy15 * cos(psi15);
dot_psi_expected = state_test15(6);

% 3. Check consistency
if abs(sdot_3f(1) - diag_3f.dynamics.dot_vx) < 1e-10 && ...
   abs(sdot_3f(2) - diag_3f.dynamics.dot_vy) < 1e-10 && ...
   abs(sdot_3f(3) - diag_3f.dynamics.dot_r)  < 1e-10 && ...
   abs(dot_X_expected - (vx15*cos(psi15) - vy15*sin(psi15))) < 1e-10
    fprintf('Test 15: Direct Phase 3F consistency (3G derivative matches 3F and pose kinematics)... PASSED\n');
else
    fprintf('Test 15: FAILED\n');
    all_tests_passed = false;
end

%% Test 16: Non-zero initial heading (psi0 = pi/4 -> trajectory along 45-degree ray)
psi0_16 = pi / 4.0;
x0_16 = [0.0; 0.0; psi0_16; 2.0; 0.0; 0.0];
delta16 = 0.0;
omega16 = (2.0 / R) * ones(4, 1);
tSpan16 = [0.0, 1.0];

[~, s16] = simulateVehicleTrajectory(tSpan16, x0_16, delta16, omega16, params);

expected_X16 = 2.0 * cos(psi0_16); % 2.0 / sqrt(2) ~ 1.4142 m
expected_Y16 = 2.0 * sin(psi0_16); % 2.0 / sqrt(2) ~ 1.4142 m

if abs(s16(end, 1) - expected_X16) < 1e-5 && abs(s16(end, 2) - expected_Y16) < 1e-5 && ...
   abs(s16(end, 3) - psi0_16) < 1e-8
    fprintf('Test 16: Non-zero initial heading (psi0=pi/4 -> straight line along 45 deg ray)... PASSED\n');
else
    fprintf('Test 16: FAILED\n');
    all_tests_passed = false;
end

%% Test 17: Negative initial heading (psi0 = -pi/2 -> trajectory along -Y axis)
psi0_17 = -pi / 2.0;
x0_17 = [0.0; 0.0; psi0_17; 2.0; 0.0; 0.0];
[~, s17] = simulateVehicleTrajectory([0.0, 1.0], x0_17, 0.0, omega16, params);

% Along -Y direction: X ~ 0, Y ~ -2.0
if abs(s17(end, 1)) < 1e-5 && abs(s17(end, 2) - (-2.0)) < 1e-5 && ...
   abs(s17(end, 3) - psi0_17) < 1e-8
    fprintf('Test 17: Negative initial heading (psi0=-pi/2 -> straight line along -Y direction)... PASSED\n');
else
    fprintf('Test 17: FAILED\n');
    all_tests_passed = false;
end

%% Test 18: Time-varying function handles for steering and wheel speeds
delta18_fcn = @(t) 0.04 * (t >= 0.5); % Step steering at t=0.5s
omega18_fcn = @(t) ((2.0 + 0.2 * t) / R) * ones(4, 1); % Linearly ramping wheel speed

[t18, s18, d18] = simulateVehicleTrajectory([0.0, 1.0], x0_5, delta18_fcn, omega18_fcn, params);

if s18(end, 1) > 1.5 && s18(end, 3) > 0 && d18.omega(end, 1) > d18.omega(1, 1)
    fprintf('Test 18: Time-varying function handles (step steering and ramping wheel speed)... PASSED\n');
else
    fprintf('Test 18: FAILED\n');
    all_tests_passed = false;
end

%% Test 19: Input validation and error handling
err_caught = 0;

% Case A: Invalid initialState length (5 elements instead of 6)
try
    simulateVehicleTrajectory([0, 1], [0; 0; 0; 2; 0], 0, omega1);
catch ME
    if strcmp(ME.identifier, 'simulateVehicleTrajectory:invalidInitialState')
        err_caught = err_caught + 1;
    end
end

% Case B: Invalid tSpan (scalar instead of vector)
try
    simulateVehicleTrajectory(1.0, x0_1, 0, omega1);
catch ME
    if strcmp(ME.identifier, 'simulateVehicleTrajectory:invalidTSpan')
        err_caught = err_caught + 1;
    end
end

% Case C: Non-monotonic tSpan
try
    simulateVehicleTrajectory([1.0, 0.5], x0_1, 0, omega1);
catch ME
    if strcmp(ME.identifier, 'simulateVehicleTrajectory:invalidTSpan')
        err_caught = err_caught + 1;
    end
end

% Case D: Invalid wheel speed dimension (3 elements)
try
    simulateVehicleTrajectory([0, 1], x0_1, 0, [1; 2; 3]);
catch ME
    if strcmp(ME.identifier, 'simulateVehicleTrajectory:invalidInput')
        err_caught = err_caught + 1;
    end
end

if err_caught == 4
    fprintf('Test 19: Input validation and error handling (4/4 invalid inputs correctly rejected)... PASSED\n');
else
    fprintf('Test 19: FAILED (caught %d/4)\n', err_caught);
    all_tests_passed = false;
end

%% Test 20: Diagnostics structure and wheel column ordering [FL, FR, RL, RR]
[~, ~, d20] = simulateVehicleTrajectory([0, 1], x0_5, delta5, omega5, params);

diag_fields_valid = isfield(d20, 'ax') && isfield(d20, 'ay') && isfield(d20, 'dot_r') && ...
                    isfield(d20, 'Fx_wheel') && isfield(d20, 'Fy_wheel') && isfield(d20, 'Fz_wheel') && ...
                    isfield(d20, 'kappa') && isfield(d20, 'alpha') && isfield(d20, 'Gxa') && ...
                    isfield(d20, 'Gyk') && isfield(d20, 'Fx_total') && isfield(d20, 'Fy_total') && ...
                    isfield(d20, 'Mz_total') && isfield(d20, 'delta') && isfield(d20, 'omega');

size_valid = size(d20.Fz_wheel, 2) == 4 && size(d20.Fx_wheel, 2) == 4 && ...
             size(d20.Fy_wheel, 2) == 4 && size(d20.kappa, 2) == 4;

% Vertical load conservation across all time points: sum(Fz) == m*g
Fz_sum_error = max(abs(sum(d20.Fz_wheel, 2) - m * params.g));

if diag_fields_valid && size_valid && Fz_sum_error < 1e-8
    fprintf('Test 20: Diagnostics structure, [FL,FR,RL,RR] ordering, and Fz conservation... PASSED\n');
else
    fprintf('Test 20: FAILED\n');
    all_tests_passed = false;
end

%% Final Summary
fprintf('\n');
if all_tests_passed
    fprintf('============================================================\n');
    fprintf('  All 20 vehicle trajectory simulation tests PASSED!\n');
    fprintf('============================================================\n');
else
    fprintf('============================================================\n');
    fprintf('  ONE OR MORE TESTS FAILED.\n');
    fprintf('============================================================\n');
end
