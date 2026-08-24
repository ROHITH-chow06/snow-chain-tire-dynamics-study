% verifyDynamicTireForces.m
% Verification script for Phase 3E: Dynamic Normal-Load / Combined-Slip Tire Force Integration

clc;
clear;
close all;

fprintf('============================================================\n');
fprintf('  Running Verification: dynamicTireForces.m\n');
fprintf('============================================================\n\n');

params = roverParameters();
R = params.R; % 0.15 m
mg_total = params.m * params.g; % 588.60 N
Fz_static_nominal = [147.15; 147.15; 147.15; 147.15];
all_tests_passed = true;
tol = 1e-10;

%% Test 1: Static Fz recovery (ax = 0, ay = 0)
vx = 2.0; vy = 0.0; r = 0.0; delta = 0.0;
omega = (vx / R) * ones(4, 1);
ax = 0.0; ay = 0.0;
[Fx, Fy, Fz, kappa, alpha, Gxa, Gyk, kin] = dynamicTireForces(vx, vy, r, delta, omega, ax, ay, params);

if max(abs(Fz - Fz_static_nominal)) < tol
    fprintf('Test 1: Static Fz recovery (ax=0, ay=0)... PASSED (all Fz = 147.1500 N)\n');
else
    fprintf('Test 1: FAILED\n');
    all_tests_passed = false;
end

%% Test 2: Dynamic Fz equals Phase 3D output
ax_test = 1.5; ay_test = -1.2;
[~, ~, Fz_dyn, ~, ~, ~, ~, ~] = dynamicTireForces(vx, vy, r, delta, omega, ax_test, ay_test, params);
Fz_3D = dynamicNormalLoadDistribution(params, ax_test, ay_test);

if max(abs(Fz_dyn - Fz_3D)) < tol
    fprintf('Test 2: Dynamic Fz matches Phase 3D output... PASSED (max diff = 0.00e+00 N)\n');
else
    fprintf('Test 2: FAILED\n');
    all_tests_passed = false;
end

%% Test 3: Four-wheel vector ordering and sizing
is_4x1 = (numel(Fx)==4 && iscolumn(Fx)) && (numel(Fy)==4 && iscolumn(Fy)) && ...
         (numel(Fz)==4 && iscolumn(Fz)) && (numel(kappa)==4 && iscolumn(kappa)) && ...
         (numel(alpha)==4 && iscolumn(alpha)) && (numel(Gxa)==4 && iscolumn(Gxa)) && ...
         (numel(Gyk)==4 && iscolumn(Gyk));

if is_4x1 && isequal(kin.wheelNames, {'FL', 'FR', 'RL', 'RR'})
    fprintf('Test 3: Four-wheel [FL, FR, RL, RR] vector dimensions and naming... PASSED\n');
else
    fprintf('Test 3: FAILED\n');
    all_tests_passed = false;
end

%% Test 4: Vertical load conservation (sum(Fz) == m*g)
if abs(sum(Fz_dyn) - mg_total) < tol
    fprintf('Test 4: Vertical load conservation... PASSED (sum(Fz) = %.2f N == m*g)\n', sum(Fz_dyn));
else
    fprintf('Test 4: FAILED\n');
    all_tests_passed = false;
end

pure_params = struct('Bx', 10.0, 'Cx', 1.90, 'Dx', 1.00, 'Ex', 0.97, ...
                     'By', 10.0, 'Cy', 1.30, 'Dy', 1.00, 'Ey', -1.00);

%% Test 5: Pure longitudinal recovery (alpha = 0 -> Gxa = 1, Fx = Fx0)
vx = 2.0; vy = 0.0; r = 0.0; delta = 0.0;
omega_drive = (2.4 / R) * ones(4, 1); % kappa = +0.20
[Fx5, ~, Fz5, kappa5, alpha5, Gxa5, ~, ~] = dynamicTireForces(vx, vy, r, delta, omega_drive, 0, 0, params);
Fx0_pure = magicFormulaLongitudinal(kappa5, Fz5, pure_params.Bx, pure_params.Cx, pure_params.Dx, pure_params.Ex);

if max(abs(alpha5)) < tol && max(abs(Gxa5 - 1.0)) < tol && max(abs(Fx5 - Fx0_pure)) < tol
    fprintf('Test 5: Pure longitudinal recovery (alpha=0 -> Gxa=1, Fx=Fx0)... PASSED\n');
else
    fprintf('Test 5: FAILED\n');
    all_tests_passed = false;
end

%% Test 6: Driving force sign (omega > vx/R -> kappa > 0 -> Fx > 0)
if all(kappa5 > 0) && all(Fx5 > 0)
    fprintf('Test 6: Driving force sign (kappa > 0 -> Fx > 0)... PASSED (all Fx > 0)\n');
else
    fprintf('Test 6: FAILED\n');
    all_tests_passed = false;
end

%% Test 7: Braking force sign (omega < vx/R -> kappa < 0 -> Fx < 0)
omega_brake = (1.6 / R) * ones(4, 1); % kappa = -0.20
[Fx7, ~, ~, kappa7, ~, ~, ~, ~] = dynamicTireForces(vx, vy, r, delta, omega_brake, 0, 0, params);

if all(kappa7 < 0) && all(Fx7 < 0)
    fprintf('Test 7: Braking force sign (kappa < 0 -> Fx < 0)... PASSED (all Fx < 0)\n');
else
    fprintf('Test 7: FAILED\n');
    all_tests_passed = false;
end

%% Test 8: Longitudinal load-transfer effect on Fx (ax > 0 -> Fx_rear > Fx_front)
ax_accel = 2.0; % Forward acceleration causes rear load gain
[Fx8, ~, Fz8, ~, ~, ~, ~, ~] = dynamicTireForces(vx, vy, r, delta, omega_drive, ax_accel, 0, params);

% With equal slip kappa = +0.20, rear wheels have higher Fz, so higher Fx
if Fz8(3) > Fz8(1) && Fz8(4) > Fz8(2) && Fx8(3) > Fx8(1) && Fx8(4) > Fx8(2)
    fprintf('Test 8: Longitudinal load transfer on Fx (ax > 0 -> Fx_rear > Fx_front)... PASSED\n');
else
    fprintf('Test 8: FAILED\n');
    all_tests_passed = false;
end

%% Test 9: Pure lateral recovery (kappa = 0, delta ~= 0 -> Gyk = 1, Fy = Fy0)
delta_steer = 0.10; % +0.10 rad steer left
% Front wheels have vx_wheel = vx*cos(delta), rear wheels have vx_wheel = vx
omega_pure_roll = [(vx * cos(delta_steer) / R); (vx * cos(delta_steer) / R); (vx / R); (vx / R)];
[~, Fy9, Fz9, kappa9, alpha9, ~, Gyk9, ~] = dynamicTireForces(vx, vy, r, delta_steer, omega_pure_roll, 0, 0, params);
Fy0_pure = magicFormulaLateral(alpha9, Fz9, pure_params.By, pure_params.Cy, pure_params.Dy, pure_params.Ey);

% Front wheels have alpha ~= 0, kappa = 0 -> Gyk = 1.0, Fy = Fy0
if max(abs(kappa9)) < tol && max(abs(Gyk9 - 1.0)) < tol && max(abs(Fy9 - Fy0_pure)) < tol
    fprintf('Test 9: Pure lateral recovery (kappa=0 -> Gyk=1, Fy=Fy0)... PASSED\n');
else
    fprintf('Test 9: FAILED\n');
    all_tests_passed = false;
end

%% Test 10: Lateral force sign (steer left delta > 0 -> vy_wheel < 0 -> alpha < 0 -> Fy > 0)
% Front wheels steered left have negative alpha, producing positive (leftward) restoring force
if alpha9(1) < 0 && alpha9(2) < 0 && Fy9(1) > 0 && Fy9(2) > 0
    fprintf('Test 10: Lateral restoring force sign (delta > 0 -> alpha < 0 -> Fy > 0)... PASSED\n');
else
    fprintf('Test 10: FAILED\n');
    all_tests_passed = false;
end

%% Test 11: Lateral load-transfer effect on Fy (ay > 0 -> |Fy_right| > |Fy_left|)
ay_turn = 2.0; % Turning left causes load shift to right (outside) wheels
[~, Fy11, Fz11, ~, ~, ~, ~, ~] = dynamicTireForces(vx, vy, r, delta_steer, omega_pure_roll, 0, ay_turn, params);

if Fz11(2) > Fz11(1) && abs(Fy11(2)) > abs(Fy11(1))
    fprintf('Test 11: Lateral load transfer on Fy (ay > 0 -> |Fy_FR| > |Fy_FL|)... PASSED\n');
else
    fprintf('Test 11: FAILED\n');
    all_tests_passed = false;
end

%% Test 12: Combined braking + cornering (kappa < 0, alpha ~= 0 -> force reduction)
[Fx12, Fy12, Fz12, kappa12, alpha12, Gxa12, Gyk12, ~] = dynamicTireForces(vx, vy, r, delta_steer, omega_brake, 0, 0, params);
Fx0_12 = magicFormulaLongitudinal(kappa12, Fz12, pure_params.Bx, pure_params.Cx, pure_params.Dx, pure_params.Ex);
Fy0_12 = magicFormulaLateral(alpha12, Fz12, pure_params.By, pure_params.Cy, pure_params.Dy, pure_params.Ey);

% Front wheels have both kappa < 0 and alpha ~= 0
front_reduced = abs(Fx12(1)) < abs(Fx0_12(1)) && abs(Fy12(1)) < abs(Fy0_12(1)) && ...
                Gxa12(1) < 1.0 && Gyk12(1) < 1.0;

if front_reduced
    fprintf('Test 12: Combined braking + cornering force reduction... PASSED (|Fx| < |Fx0|, |Fy| < |Fy0|)\n');
else
    fprintf('Test 12: FAILED\n');
    all_tests_passed = false;
end

%% Test 13: Combined driving + cornering (kappa > 0, alpha ~= 0 -> force reduction)
[Fx13, Fy13, Fz13, kappa13, alpha13, Gxa13, Gyk13, ~] = dynamicTireForces(vx, vy, r, delta_steer, omega_drive, 0, 0, params);
Fx0_13 = magicFormulaLongitudinal(kappa13, Fz13, pure_params.Bx, pure_params.Cx, pure_params.Dx, pure_params.Ex);
Fy0_13 = magicFormulaLateral(alpha13, Fz13, pure_params.By, pure_params.Cy, pure_params.Dy, pure_params.Ey);

front_reduced_drive = abs(Fx13(1)) < abs(Fx0_13(1)) && abs(Fy13(1)) < abs(Fy0_13(1)) && ...
                      Gxa13(1) < 1.0 && Gyk13(1) < 1.0;

if front_reduced_drive
    fprintf('Test 13: Combined driving + cornering force reduction... PASSED (|Fx| < |Fx0|, |Fy| < |Fy0|)\n');
else
    fprintf('Test 13: FAILED\n');
    all_tests_passed = false;
end

%% Test 14: Combined-slip weighting factor consistency
Gxa_direct = combinedSlipWeighting(alpha13, 10.0, 1.0, -1.0, 0.0);
Gyk_direct = combinedSlipWeighting(kappa13, 10.0, 1.0, -1.0, 0.0);

if max(abs(Gxa13 - Gxa_direct)) < tol && max(abs(Gyk13 - Gyk_direct)) < tol
    fprintf('Test 14: Combined-slip weighting factor consistency... PASSED (Gxa & Gyk match weighting function)\n');
else
    fprintf('Test 14: FAILED\n');
    all_tests_passed = false;
end

%% Test 15: Four-quadrant force signs consistency
% Evaluate (+kappa, -alpha), (+kappa, +alpha), (-kappa, -alpha), (-kappa, +alpha)
test_quad_passed = true;
deltas_test = [0.10, -0.10, 0.10, -0.10];
omegas_test = [(2.4/R)*ones(4,1), (2.4/R)*ones(4,1), (1.6/R)*ones(4,1), (1.6/R)*ones(4,1)];
expected_Fx_sign = [1, 1, -1, -1];
expected_Fy_sign = [1, -1, 1, -1]; % delta > 0 -> alpha < 0 -> Fy > 0; delta < 0 -> alpha > 0 -> Fy < 0

for q = 1:4
    [Fx_q, Fy_q, ~, ~, ~, ~, ~, ~] = dynamicTireForces(vx, vy, r, deltas_test(q), omegas_test(:, q), 0, 0, params);
    if sign(Fx_q(1)) ~= expected_Fx_sign(q) || sign(Fy_q(1)) ~= expected_Fy_sign(q)
        test_quad_passed = false;
    end
end

if test_quad_passed
    fprintf('Test 15: Four-quadrant combined force signs consistency... PASSED\n');
else
    fprintf('Test 15: FAILED\n');
    all_tests_passed = false;
end

%% Test 16: Vertical load conservation across acceleration grid
grid_accel_passed = true;
[AX, AY] = ndgrid(linspace(-2.0, 2.0, 5), linspace(-2.0, 2.0, 5));
for g = 1:numel(AX)
    [~, ~, Fz_g, ~, ~, ~, ~, ~] = dynamicTireForces(vx, vy, r, delta, omega, AX(g), AY(g), params);
    if abs(sum(Fz_g) - mg_total) > tol
        grid_accel_passed = false;
    end
end

if grid_accel_passed
    fprintf('Test 16: Vertical load conservation across (ax, ay) grid... PASSED (sum(Fz) == m*g everywhere)\n');
else
    fprintf('Test 16: FAILED\n');
    all_tests_passed = false;
end

%% Test 17: Symmetry under zero lateral acceleration (ay = 0, delta = 0, vy = 0, r = 0)
[Fx17, Fy17, Fz17, ~, ~, ~, ~, ~] = dynamicTireForces(vx, 0, 0, 0, omega_drive, 1.5, 0, params);

if abs(Fz17(1) - Fz17(2)) < tol && abs(Fz17(3) - Fz17(4)) < tol && ...
   abs(Fx17(1) - Fx17(2)) < tol && abs(Fx17(3) - Fx17(4)) < tol && ...
   max(abs(Fy17)) < tol
    fprintf('Test 17: Left/right symmetry under zero lateral acceleration... PASSED\n');
else
    fprintf('Test 17: FAILED\n');
    all_tests_passed = false;
end

%% Test 18: Symmetry under zero longitudinal acceleration (ax = 0, symmetric rover)
[~, ~, Fz18, ~, ~, ~, ~, ~] = dynamicTireForces(vx, 0, 0, 0, omega, 0, 0, params);

if abs(Fz18(1) - Fz18(3)) < tol && abs(Fz18(2) - Fz18(4)) < tol
    fprintf('Test 18: Front/rear symmetry under zero longitudinal acceleration... PASSED\n');
else
    fprintf('Test 18: FAILED\n');
    all_tests_passed = false;
end

%% Test 19: Wheel-lift rejection propagation (ay = 12 m/s^2)
wheel_lift_caught = false;
try
    dynamicTireForces(vx, vy, r, delta, omega, 0, 12.0, params);
catch ME
    if strcmp(ME.identifier, 'dynamicNormalLoadDistribution:negativeNormalLoad')
        wheel_lift_caught = true;
    end
end

if wheel_lift_caught
    fprintf('Test 19: Wheel-lift rejection propagation... PASSED (negative normal load error caught)\n');
else
    fprintf('Test 19: FAILED\n');
    all_tests_passed = false;
end

%% Test 20: Standstill equilibrium (vx=0, vy=0, r=0, omega=0, delta=0, ax=0, ay=0)
[Fx20, Fy20, Fz20, kappa20, alpha20, ~, ~, ~] = dynamicTireForces(0, 0, 0, 0, zeros(4,1), 0, 0, params);

if max(abs(Fx20)) < tol && max(abs(Fy20)) < tol && max(abs(Fz20 - Fz_static_nominal)) < tol && ...
   max(abs(kappa20)) < tol && max(abs(alpha20)) < tol
    fprintf('Test 20: Standstill equilibrium (all velocities=0 -> Fx=0, Fy=0, Fz=Fz_static)... PASSED\n');
else
    fprintf('Test 20: FAILED\n');
    all_tests_passed = false;
end

fprintf('\n');
if all_tests_passed
    fprintf('============================================================\n');
    fprintf('  All 20 dynamic tire force integration tests PASSED!\n');
    fprintf('============================================================\n');
else
    fprintf('============================================================\n');
    fprintf('  ONE OR MORE TESTS FAILED.\n');
    fprintf('============================================================\n');
end
