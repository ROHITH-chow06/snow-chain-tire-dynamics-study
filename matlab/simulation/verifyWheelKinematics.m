% verifyWheelKinematics.m
% Verification script for four-wheel rover kinematics and slip angles.
%
% Tests:
%   1. Stationary rover (vx=0, vy=0, r=0): zero velocities, regularized slip, alpha=0, no NaN/Inf.
%   2. Straight-line forward motion: identical wheel longitudinal velocities, zero slip angles.
%   3. Pure yaw motion (r > 0): differential left/right longitudinal and front/rear lateral velocities.
%   4. Straight motion with front steering: wheel-frame velocity rotation, signed slip angle (alpha < 0).
%   5. Four-wheel dimensional consistency and ordering [FL, FR, RL, RR].
%   6. Direct wheelSlipAngle unit tests (regularization, quadrants, vectorization, error handling).

clear;
clc;

fprintf('====================================================\n');
fprintf('  Running Verification: Four-Wheel Rover Kinematics\n');
fprintf('====================================================\n\n');

params = roverParameters();
tol = 1e-12;

%% Test 1: Stationary Rover
fprintf('Test 1: Stationary rover (vx=0, vy=0, r=0)... ');
vx_1 = 0.0; vy_1 = 0.0; r_1 = 0.0;
delta_1 = 0.0;
omega_1 = [0.0; 0.0; 0.0; 0.0];

kin_1 = wheelKinematics(vx_1, vy_1, r_1, delta_1, omega_1, params);

assert(all(kin_1.vx_body == 0), 'Test 1 Failed: Body vx not zero.');
assert(all(kin_1.vy_body == 0), 'Test 1 Failed: Body vy not zero.');
assert(all(kin_1.vx_wheel == 0), 'Test 1 Failed: Wheel vx not zero.');
assert(all(kin_1.vy_wheel == 0), 'Test 1 Failed: Wheel vy not zero.');
assert(all(isfinite(kin_1.kappa)), 'Test 1 Failed: Non-finite kappa.');
assert(all(abs(kin_1.kappa) < tol), 'Test 1 Failed: Standstill kappa should be 0.');
assert(all(isfinite(kin_1.alpha)), 'Test 1 Failed: Non-finite alpha.');
assert(all(abs(kin_1.alpha) < tol), 'Test 1 Failed: Standstill alpha should be 0.');
fprintf('PASSED (all velocities=0, kappa=0, alpha=0, no NaN/Inf)\n');

%% Test 2: Straight-Line Forward Motion
fprintf('Test 2: Straight-line motion (vx=2.0 m/s, vy=0, r=0)... ');
vx_2 = 2.0; vy_2 = 0.0; r_2 = 0.0;
delta_2 = 0.0;

% Pure rolling wheel speeds: omega = vx / R = 2.0 / 0.15 = 13.333 rad/s
omega_2_roll = (vx_2 / params.R) * ones(4, 1);
kin_2_roll = wheelKinematics(vx_2, vy_2, r_2, delta_2, omega_2_roll, params);

assert(all(abs(kin_2_roll.vx_body - vx_2) < tol), 'Test 2 Failed: Body vx mismatch.');
assert(all(abs(kin_2_roll.vy_body) < tol), 'Test 2 Failed: Body vy not zero.');
assert(all(abs(kin_2_roll.vx_wheel - vx_2) < tol), 'Test 2 Failed: Wheel vx mismatch.');
assert(all(abs(kin_2_roll.vy_wheel) < tol), 'Test 2 Failed: Wheel vy not zero.');
assert(all(abs(kin_2_roll.kappa) < tol), 'Test 2 Failed: Pure rolling kappa should be 0.');
assert(all(abs(kin_2_roll.alpha) < tol), 'Test 2 Failed: Straight motion alpha should be 0.');

% Driving slip test: omega = 16.0 rad/s -> R*omega = 2.4 m/s -> kappa = (2.4 - 2.0)/2.0 = 0.20
omega_2_drive = 16.0 * ones(4, 1);
kin_2_drive = wheelKinematics(vx_2, vy_2, r_2, delta_2, omega_2_drive, params);
expected_kappa = (params.R * 16.0 - vx_2) / vx_2; % 0.20

assert(all(abs(kin_2_drive.kappa - expected_kappa) < tol), ...
    'Test 2 Failed: Driving slip ratio mismatch.');
fprintf('PASSED (all vx=2.0 m/s, pure rolling kappa=0, driving kappa=+0.2000)\n');

%% Test 3: Pure Yaw Motion (Spin about CG)
fprintf('Test 3: Pure yaw motion (r = +1.0 rad/s, CCW)... ');
vx_3 = 0.0; vy_3 = 0.0; r_3 = 1.0;
delta_3 = 0.0;
omega_3 = zeros(4, 1);

kin_3 = wheelKinematics(vx_3, vy_3, r_3, delta_3, omega_3, params);

% Theoretical velocities:
% FL (x=+0.40, y=+0.30): vx_body = -r*y = -0.30, vy_body = +r*x = +0.40
% FR (x=+0.40, y=-0.30): vx_body = -r*y = +0.30, vy_body = +r*x = +0.40
% RL (x=-0.40, y=+0.30): vx_body = -r*y = -0.30, vy_body = +r*x = -0.40
% RR (x=-0.40, y=-0.30): vx_body = -r*y = +0.30, vy_body = +r*x = -0.40
expected_vx_body = [-r_3 * (params.tf/2); +r_3 * (params.tf/2); -r_3 * (params.tr/2); +r_3 * (params.tr/2)];
expected_vy_body = [+r_3 * params.lf;     +r_3 * params.lf;     -r_3 * params.lr;     -r_3 * params.lr];

assert(all(abs(kin_3.vx_body - expected_vx_body) < tol), 'Test 3 Failed: Yaw vx_body mismatch.');
assert(all(abs(kin_3.vy_body - expected_vy_body) < tol), 'Test 3 Failed: Yaw vy_body mismatch.');

% Verify left/right differential signs
assert(kin_3.vx_body(1) < 0 && kin_3.vx_body(2) > 0, 'Test 3 Failed: Front left/right vx sign error.');
assert(kin_3.vx_body(3) < 0 && kin_3.vx_body(4) > 0, 'Test 3 Failed: Rear left/right vx sign error.');
fprintf('PASSED (left wheels vx=-0.30 m/s, right wheels vx=+0.30 m/s)\n');

%% Test 4: Straight Driving with Positive Front Steering
fprintf('Test 4: Straight motion with positive steer (delta = +0.10 rad, vx = 2.0 m/s)... ');
vx_4 = 2.0; vy_4 = 0.0; r_4 = 0.0;
delta_4 = 0.10; % +0.10 rad (~5.73 deg)
omega_4 = (vx_4 / params.R) * ones(4, 1);

kin_4 = wheelKinematics(vx_4, vy_4, r_4, delta_4, omega_4, params);

% Front wheels (steered by delta):
% vx_wheel =  cos(delta)*vx_body + sin(delta)*vy_body = 2.0 * cos(0.10)
% vy_wheel = -sin(delta)*vx_body + cos(delta)*vy_body = -2.0 * sin(0.10)
expected_vx_w_front = vx_4 * cos(delta_4);
expected_vy_w_front = -vx_4 * sin(delta_4);
expected_alpha_front = atan2(expected_vy_w_front, expected_vx_w_front); % -delta_4 = -0.10 rad

assert(abs(kin_4.vx_wheel(1) - expected_vx_w_front) < tol, 'Test 4 Failed: FL vx_wheel mismatch.');
assert(abs(kin_4.vx_wheel(2) - expected_vx_w_front) < tol, 'Test 4 Failed: FR vx_wheel mismatch.');
assert(abs(kin_4.vy_wheel(1) - expected_vy_w_front) < tol, 'Test 4 Failed: FL vy_wheel mismatch.');
assert(abs(kin_4.vy_wheel(2) - expected_vy_w_front) < tol, 'Test 4 Failed: FR vy_wheel mismatch.');
assert(abs(kin_4.alpha(1) - expected_alpha_front) < tol, 'Test 4 Failed: FL alpha mismatch.');
assert(abs(kin_4.alpha(2) - expected_alpha_front) < tol, 'Test 4 Failed: FR alpha mismatch.');
assert(kin_4.alpha(1) < 0, 'Test 4 Failed: Steered left front wheel slip angle should be negative.');

% Rear wheels (unsteered, delta = 0):
assert(abs(kin_4.vx_wheel(3) - vx_4) < tol, 'Test 4 Failed: RL vx_wheel mismatch.');
assert(abs(kin_4.vx_wheel(4) - vx_4) < tol, 'Test 4 Failed: RR vx_wheel mismatch.');
assert(abs(kin_4.vy_wheel(3)) < tol, 'Test 4 Failed: RL vy_wheel not zero.');
assert(abs(kin_4.vy_wheel(4)) < tol, 'Test 4 Failed: RR vy_wheel not zero.');
assert(abs(kin_4.alpha(3)) < tol, 'Test 4 Failed: RL alpha should be 0.');
assert(abs(kin_4.alpha(4)) < tol, 'Test 4 Failed: RR alpha should be 0.');
fprintf('PASSED (front alpha = -0.1000 rad, rear alpha = 0.0000 rad)\n');

%% Test 5: Dimensional Consistency and Wheel Ordering
fprintf('Test 5: Four-wheel dimensional consistency and ordering... ');
assert(isequal(size(kin_4.vx_body), [4, 1]), 'Test 5 Failed: vx_body size.');
assert(isequal(size(kin_4.vy_body), [4, 1]), 'Test 5 Failed: vy_body size.');
assert(isequal(size(kin_4.vx_wheel), [4, 1]), 'Test 5 Failed: vx_wheel size.');
assert(isequal(size(kin_4.vy_wheel), [4, 1]), 'Test 5 Failed: vy_wheel size.');
assert(isequal(size(kin_4.kappa), [4, 1]), 'Test 5 Failed: kappa size.');
assert(isequal(size(kin_4.alpha), [4, 1]), 'Test 5 Failed: alpha size.');
assert(isequal(kin_4.wheelNames, {'FL', 'FR', 'RL', 'RR'}), 'Test 5 Failed: Wheel ordering mismatch.');
fprintf('PASSED (all 4x1 vectors with [FL, FR, RL, RR] ordering)\n');

%% Test 6: Direct wheelSlipAngle Function Unit Tests
fprintf('Test 6: wheelSlipAngle unit tests (quadrants, regularization, errors)... ');
% Standstill
a_zero = wheelSlipAngle(0.0, 0.0);
assert(a_zero == 0.0, 'Test 6 Failed: Standstill slip angle should be 0.0.');

% Below threshold
a_subthresh = wheelSlipAngle(1e-5, 1e-5, 1e-4);
assert(a_subthresh == 0.0, 'Test 6 Failed: Sub-threshold slip angle should be regularized to 0.0.');

% Quadrant 1 (vx > 0, vy > 0) -> alpha > 0
a_q1 = wheelSlipAngle(1.0, 1.0);
assert(abs(a_q1 - pi/4) < tol, 'Test 6 Failed: Quadrant 1 angle should be +pi/4.');

% Quadrant 4 (vx > 0, vy < 0) -> alpha < 0
a_q4 = wheelSlipAngle(-1.0, 1.0);
assert(abs(a_q4 - (-pi/4)) < tol, 'Test 6 Failed: Quadrant 4 angle should be -pi/4.');

% Vectorized execution
vy_vec = [0.0; -1.0; 1.0; 0.0];
vx_vec = [2.0;  2.0; 2.0; 0.0];
a_vec = wheelSlipAngle(vy_vec, vx_vec);
assert(numel(a_vec) == 4, 'Test 6 Failed: Vector output size mismatch.');
assert(a_vec(1) == 0.0 && a_vec(2) < 0 && a_vec(3) > 0 && a_vec(4) == 0.0, ...
    'Test 6 Failed: Vectorized quadrant evaluation error.');

% Dimension mismatch error handling
threwError = false;
try
    wheelSlipAngle([1; 2; 3], [1; 2]);
catch ME
    threwError = true;
    assert(strcmp(ME.identifier, 'wheelSlipAngle:dimensionMismatch'), ...
        'Test 6 Failed: Unexpected error identifier.');
end
assert(threwError, 'Test 6 Failed: Mismatched dimensions were not rejected.');
fprintf('PASSED (regularization, quadrants, vectorization, error handling verified)\n');

%% Summary
fprintf('\n====================================================\n');
fprintf('  All 6 Kinematics Verification Tests PASSED!\n');
fprintf('====================================================\n');
