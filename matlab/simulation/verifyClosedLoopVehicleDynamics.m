% verifyClosedLoopVehicleDynamics.m
% Verification script for Phase 3F: Closed-Loop Four-Wheel Vehicle Dynamics

clc;
clear;
close all;

fprintf('============================================================\n');
fprintf('  Running Verification: Closed-Loop Vehicle Dynamics (3F)\n');
fprintf('============================================================\n\n');

params = roverParameters();
R = params.R; % 0.15 m
m = params.m; % 60.0 kg
Iz = params.Iz; % 5.0 kg*m^2
Fz_static_nominal = [147.15; 147.15; 147.15; 147.15];
all_tests_passed = true;
tol = 1e-6;

%% Test 1: Standstill equilibrium (v=0, omega=0 -> dot_x = 0, a = 0, Fz = static)
state_0 = [0.0; 0.0; 0.0];
delta_0 = 0.0;
omega_0 = zeros(4, 1);
[sdot1, d1] = closedLoopVehicleDerivative(state_0, delta_0, omega_0, params);

if max(abs(sdot1)) < 1e-10 && abs(d1.ax) < 1e-10 && abs(d1.ay) < 1e-10 && ...
   max(abs(d1.Fz_wheel - Fz_static_nominal)) < 1e-10
    fprintf('Test 1: Standstill equilibrium (v=0, omega=0 -> dot_x=0, Fz=static)... PASSED\n');
else
    fprintf('Test 1: FAILED\n');
    all_tests_passed = false;
end

%% Test 2: Pure symmetric traction (omega > vx/R -> ax > 0, dot_vx > 0, dot_vy = 0, dot_r = 0)
state_2 = [2.0; 0.0; 0.0];
delta_2 = 0.0;
omega_2 = (2.4 / R) * ones(4, 1); % kappa = +0.20
[sdot2, d2] = closedLoopVehicleDerivative(state_2, delta_2, omega_2, params);

if d2.ax > 0 && sdot2(1) > 0 && abs(sdot2(2)) < 1e-10 && abs(sdot2(3)) < 1e-10
    fprintf('Test 2: Pure symmetric traction (ax > 0, dot_vx > 0, dot_vy=0, dot_r=0)... PASSED\n');
else
    fprintf('Test 2: FAILED\n');
    all_tests_passed = false;
end

%% Test 3: Pure symmetric braking (omega < vx/R -> ax < 0, dot_vx < 0, dot_vy = 0, dot_r = 0)
state_3 = [2.0; 0.0; 0.0];
delta_3 = 0.0;
omega_3 = (1.6 / R) * ones(4, 1); % kappa = -0.20
[sdot3, d3] = closedLoopVehicleDerivative(state_3, delta_3, omega_3, params);

if d3.ax < 0 && sdot3(1) < 0 && abs(sdot3(2)) < 1e-10 && abs(sdot3(3)) < 1e-10
    fprintf('Test 3: Pure symmetric braking (ax < 0, dot_vx < 0, dot_vy=0, dot_r=0)... PASSED\n');
else
    fprintf('Test 3: FAILED\n');
    all_tests_passed = false;
end

%% Test 4: Zero-force constant straight-line state (omega = vx/R -> ax=0, ay=0, dot_x=0)
state_4 = [3.0; 0.0; 0.0];
delta_4 = 0.0;
omega_4 = (3.0 / R) * ones(4, 1);
[sdot4, d4] = closedLoopVehicleDerivative(state_4, delta_4, omega_4, params);

if max(abs(sdot4)) < 1e-10 && abs(d4.ax) < 1e-10 && abs(d4.ay) < 1e-10
    fprintf('Test 4: Zero-force constant straight-line state (dot_x=0, ax=0, ay=0)... PASSED\n');
else
    fprintf('Test 4: FAILED\n');
    all_tests_passed = false;
end

%% Test 5: Positive steering response (delta > 0 -> ay > 0, dot_r > 0)
state_5 = [2.0; 0.0; 0.0];
delta_5 = 0.08;
omega_5 = [(2.0 * cos(delta_5) / R); (2.0 * cos(delta_5) / R); (2.0 / R); (2.0 / R)];
[sdot5, d5] = closedLoopVehicleDerivative(state_5, delta_5, omega_5, params);

if d5.ay > 0 && sdot5(3) > 0 && d5.dynamics.Mz_total > 0
    fprintf('Test 5: Positive steering response (delta > 0 -> ay > 0, dot_r > 0)... PASSED\n');
else
    fprintf('Test 5: FAILED\n');
    all_tests_passed = false;
end

%% Test 6: Negative steering response (delta < 0 -> ay < 0, dot_r < 0)
delta_6 = -0.08;
omega_6 = [(2.0 * cos(delta_6) / R); (2.0 * cos(delta_6) / R); (2.0 / R); (2.0 / R)];
[sdot6, d6] = closedLoopVehicleDerivative(state_5, delta_6, omega_6, params);

if d6.ay < 0 && sdot6(3) < 0 && d6.dynamics.Mz_total < 0
    fprintf('Test 6: Negative steering response (delta < 0 -> ay < 0, dot_r < 0)... PASSED\n');
else
    fprintf('Test 6: FAILED\n');
    all_tests_passed = false;
end

%% Test 7: Differential longitudinal force yaw moment (omega_left < omega_right -> Mz > 0, dot_r > 0)
state_7 = [2.0; 0.0; 0.0];
delta_7 = 0.0;
omega_7 = [(1.8 / R); (2.2 / R); (1.8 / R); (2.2 / R)]; % Left braking/coasting, right driving
[sdot7, d7] = closedLoopVehicleDerivative(state_7, delta_7, omega_7, params);

if d7.dynamics.Mz_total > 0 && sdot7(3) > 0
    fprintf('Test 7: Differential longitudinal traction (omega_L < omega_R -> Mz > 0, dot_r > 0)... PASSED\n');
else
    fprintf('Test 7: FAILED\n');
    all_tests_passed = false;
end

%% Test 8: Longitudinal load transfer coupling (ax > 0 -> Fz_rear > Fz_front)
if d2.Fz_wheel(3) > d2.Fz_wheel(1) && d2.Fz_wheel(4) > d2.Fz_wheel(2)
    fprintf('Test 8: Longitudinal load transfer coupling (ax > 0 -> Fz_rear > Fz_front)... PASSED\n');
else
    fprintf('Test 8: FAILED\n');
    all_tests_passed = false;
end

%% Test 9: Lateral load transfer coupling (ay > 0 -> Fz_right > Fz_left)
if d5.Fz_wheel(2) > d5.Fz_wheel(1) && d5.Fz_wheel(4) > d5.Fz_wheel(3)
    fprintf('Test 9: Lateral load transfer coupling (ay > 0 -> Fz_FR > Fz_FL, Fz_RR > Fz_RL)... PASSED\n');
else
    fprintf('Test 9: FAILED\n');
    all_tests_passed = false;
end

%% Test 10: Combined acceleration 4-wheel asymmetric Fz (ax > 0, ay > 0 -> Fz_RR > Fz_FL)
state_10 = [2.0; 0.0; 0.0];
delta_10 = 0.06;
omega_10 = [(2.4 / R); (2.4 / R); (2.4 / R); (2.4 / R)];
[~, d10] = closedLoopVehicleDerivative(state_10, delta_10, omega_10, params);

if d10.ax > 0 && d10.ay > 0 && d10.Fz_wheel(4) > d10.Fz_wheel(1)
    fprintf('Test 10: Combined load transfer (ax > 0, ay > 0 -> Fz_RR highest, Fz_FL lowest)... PASSED\n');
else
    fprintf('Test 10: FAILED\n');
    all_tests_passed = false;
end

%% Test 11: Combined-slip force reduction in closed loop
if d10.Gxa(1) < 1.0 && d10.Gyk(1) < 1.0
    fprintf('Test 11: Combined-slip force reduction in closed loop (Gxa < 1, Gyk < 1)... PASSED\n');
else
    fprintf('Test 11: FAILED\n');
    all_tests_passed = false;
end

%% Test 12: Force transformation consistency (sum(Fx_body) == m*ax, sum(Fy_body) == m*ay)
if abs(d10.dynamics.Fx_total - m * d10.ax) < 1e-10 && ...
   abs(d10.dynamics.Fy_total - m * d10.ay) < 1e-10
    fprintf('Test 12: Body force transformation consistency (Fx_total = m*ax, Fy_total = m*ay)... PASSED\n');
else
    fprintf('Test 12: FAILED\n');
    all_tests_passed = false;
end

%% Test 13: Yaw moment consistency (Mz_total == Iz * dot_r)
if abs(d10.dynamics.Mz_total - Iz * d10.dot_r) < 1e-10
    fprintf('Test 13: Yaw moment balance consistency (Mz_total = Iz * dot_r)... PASSED\n');
else
    fprintf('Test 13: FAILED\n');
    all_tests_passed = false;
end

%% Test 14: Fixed-point convergence across diverse states
fp_tests_passed = true;
test_states = [
    1.0,  0.0,  0.0,  0.04, 1.1;
    2.0,  0.1,  0.2,  0.05, 1.1;
    2.0, -0.1, -0.1, -0.04, 0.9;
    1.5,  0.1,  0.1,  0.00, 1.2
];

for i = 1:size(test_states, 1)
    v_x = test_states(i, 1);
    v_y = test_states(i, 2);
    om_factor = test_states(i, 5);
    om = (v_x * om_factor / R) * ones(4, 1);
    [~, d_fp] = closedLoopVehicleDerivative([v_x; v_y; test_states(i,3)], test_states(i,4), om, params);
    if ~d_fp.solver_diag.converged || d_fp.solver_diag.final_error > 1e-6
        fp_tests_passed = false;
    end
end

if fp_tests_passed
    fprintf('Test 14: Fixed-point convergence across diverse states (all converged < 1e-6)... PASSED\n');
else
    fprintf('Test 14: FAILED\n');
    all_tests_passed = false;
end

%% Test 15: Convergence tolerance sensitivity (tight tolerance 1e-10)
opts_tight.tol = 1e-10;
opts_tight.max_iter = 50;
[~, d15] = closedLoopVehicleDerivative(state_10, delta_10, omega_10, params, [], [], opts_tight);

if d15.solver_diag.converged && d15.solver_diag.final_error < 1e-10
    fprintf('Test 15: Convergence tolerance sensitivity (converged to < 1e-10 in %d iters)... PASSED\n', ...
            d15.solver_diag.iterations);
else
    fprintf('Test 15: FAILED\n');
    all_tests_passed = false;
end

%% Test 16: Deterministic maximum iteration handling (max_iter = 1 on non-converged step)
opts_maxiter.tol = 1e-10;
opts_maxiter.max_iter = 1;
opts_maxiter.a_init = [0.0; 0.0];
max_iter_caught = false;

try
    closedLoopVehicleDerivative(state_10, delta_10, omega_10, params, [], [], opts_maxiter);
catch ME
    if strcmp(ME.identifier, 'solveQuasiStaticForceBalance:maxIterationsExceeded')
        max_iter_caught = true;
    end
end

if max_iter_caught
    fprintf('Test 16: Deterministic max_iter rejection (maxIterationsExceeded error caught)... PASSED\n');
else
    fprintf('Test 16: FAILED\n');
    all_tests_passed = false;
end

%% Test 17: Wheel-lift rejection propagation
% Provide initial acceleration guess exceeding rollover/wheel-lift limit
opts_lift.a_init = [0.0; 12.0]; % ay = 12 m/s^2 produces Fz_FL < 0
wheel_lift_caught = false;

try
    closedLoopVehicleDerivative(state_10, delta_10, omega_10, params, [], [], opts_lift);
catch ME
    if strcmp(ME.identifier, 'dynamicNormalLoadDistribution:negativeNormalLoad')
        wheel_lift_caught = true;
    end
end

if wheel_lift_caught
    fprintf('Test 17: Wheel-lift rejection propagation (negativeNormalLoad error caught)... PASSED\n');
else
    fprintf('Test 17: FAILED\n');
    all_tests_passed = false;
end

%% Test 18: Numerical robustness (no NaN/Inf across grid)
grid_robust = true;
[VX, OM_FAC] = ndgrid([0.5, 1.5, 3.0], [0.8, 1.0, 1.2]);
for g = 1:numel(VX)
    s_g = [VX(g); 0.0; 0.0];
    om_g = (VX(g) * OM_FAC(g) / R) * ones(4, 1);
    [sdot_g, d_g] = closedLoopVehicleDerivative(s_g, 0.05, om_g, params);
    if any(isnan(sdot_g)) || any(isinf(sdot_g)) || ...
       any(isnan(d_g.Fz_wheel)) || any(isinf(d_g.Fz_wheel))
        grid_robust = false;
    end
end

if grid_robust
    fprintf('Test 18: Numerical robustness across (vx, omega) grid (no NaN/Inf)... PASSED\n');
else
    fprintf('Test 18: FAILED\n');
    all_tests_passed = false;
end

%% Test 19: Left/right symmetry at zero lateral acceleration
[~, d19] = closedLoopVehicleDerivative([2.0; 0.0; 0.0], 0.0, omega_2, params);

if abs(d19.Fz_wheel(1) - d19.Fz_wheel(2)) < 1e-10 && ...
   abs(d19.Fz_wheel(3) - d19.Fz_wheel(4)) < 1e-10 && ...
   abs(d19.Fx_wheel(1) - d19.Fx_wheel(2)) < 1e-10 && ...
   abs(d19.Fx_wheel(3) - d19.Fx_wheel(4)) < 1e-10
    fprintf('Test 19: Left/right symmetry at zero lateral acceleration... PASSED\n');
else
    fprintf('Test 19: FAILED\n');
    all_tests_passed = false;
end

%% Test 20: Front/rear symmetry for symmetric rover at standstill
if abs(d1.Fz_wheel(1) - d1.Fz_wheel(3)) < 1e-10 && ...
   abs(d1.Fz_wheel(2) - d1.Fz_wheel(4)) < 1e-10
    fprintf('Test 20: Front/rear symmetry for symmetric rover at standstill... PASSED\n');
else
    fprintf('Test 20: FAILED\n');
    all_tests_passed = false;
end

%% Test 21: Consistency with Phase 3E (dynamicTireForces at converged ax, ay)
[Fx_3e, Fy_3e, Fz_3e, ~, ~, ~, ~, ~] = dynamicTireForces(state_10(1), state_10(2), state_10(3), ...
                                                          delta_10, omega_10, d10.ax, d10.ay, params);

if max(abs(d10.Fx_wheel - Fx_3e)) < tol && ...
   max(abs(d10.Fy_wheel - Fy_3e)) < tol && ...
   max(abs(d10.Fz_wheel - Fz_3e)) < tol
    fprintf('Test 21: Consistency with Phase 3E evaluated at converged (ax, ay)... PASSED\n');
else
    fprintf('Test 21: FAILED\n');
    all_tests_passed = false;
end

%% Test 22: Complete force/moment-to-acceleration consistency
dot_vx_expected = d10.ax + state_10(2) * state_10(3); % ax + vy * r
dot_vy_expected = d10.ay - state_10(1) * state_10(3); % ay - vx * r
dot_r_expected  = d10.dynamics.Mz_total / Iz;

[sdot22, d22] = closedLoopVehicleDerivative(state_10, delta_10, omega_10, params);

if abs(sdot22(1) - dot_vx_expected) < 1e-10 && ...
   abs(sdot22(2) - dot_vy_expected) < 1e-10 && ...
   abs(sdot22(3) - dot_r_expected) < 1e-10
    fprintf('Test 22: Complete 3-DOF kinematic/dynamic acceleration consistency... PASSED\n');
else
    fprintf('Test 22: FAILED\n');
    all_tests_passed = false;
end

fprintf('\n');
if all_tests_passed
    fprintf('============================================================\n');
    fprintf('  All 22 closed-loop vehicle dynamics tests PASSED!\n');
    fprintf('============================================================\n');
else
    fprintf('============================================================\n');
    fprintf('  ONE OR MORE TESTS FAILED.\n');
    fprintf('============================================================\n');
end
