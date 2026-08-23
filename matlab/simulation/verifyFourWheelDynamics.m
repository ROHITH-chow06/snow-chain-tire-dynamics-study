% verifyFourWheelDynamics.m
% Verification script for four-wheel rover dynamics, load distribution,
% force transformation, yaw moment, and 3-DOF planar equations of motion.
%
% Tests:
%   1. Static normal-load distribution (equilibrium and axle formulas).
%   2. Force transformation from wheel coordinates to vehicle body frame.
%   3. Individual wheel yaw moment calculation against analytical formula.
%   4. Pure symmetric forward tractive acceleration (dot(vx) > 0, dot(r) = 0).
%   5. Differential longitudinal tractive forces generating pure yaw acceleration (dot(r) > 0).
%   6. Kinematic acceleration coupling terms (vy*r and -vx*r).
%   7. Standstill equilibrium (vx=0, vy=0, r=0, zero forces -> dvx=0, dvy=0, dr=0).
%   8. Constant straight-line state (vx>0, vy=0, r=0, zero forces -> dvx=0, dvy=0, dr=0).

clear;
clc;

fprintf('====================================================\n');
fprintf('  Running Verification: Four-Wheel Rover Dynamics\n');
fprintf('====================================================\n\n');

params = roverParameters();
tol = 1e-12;

%% Test 1: Static Normal-Load Distribution
fprintf('Test 1: Static normal-load distribution... ');
[Fz, Fz_axles] = normalLoadDistribution(params);

expected_total_Fz = params.m * params.g; % 60 * 9.81 = 588.6 N
expected_front_Fz = expected_total_Fz * (params.lr / (params.lf + params.lr)); % 294.3 N
expected_rear_Fz  = expected_total_Fz * (params.lf / (params.lf + params.lr)); % 294.3 N

assert(abs(sum(Fz) - expected_total_Fz) < tol, ...
    'Test 1 Failed: Sum of normal loads does not equal m*g.');
assert(abs(Fz_axles.total - expected_total_Fz) < tol, ...
    'Test 1 Failed: Total axle load mismatch.');
assert(abs(Fz_axles.front - expected_front_Fz) < tol, ...
    'Test 1 Failed: Front axle load mismatch.');
assert(abs(Fz_axles.rear - expected_rear_Fz) < tol, ...
    'Test 1 Failed: Rear axle load mismatch.');
assert(abs(Fz(1) - expected_front_Fz/2) < tol, 'Test 1 Failed: Fz_FL mismatch.');
assert(abs(Fz(2) - expected_front_Fz/2) < tol, 'Test 1 Failed: Fz_FR mismatch.');
assert(abs(Fz(3) - expected_rear_Fz/2) < tol,  'Test 1 Failed: Fz_RL mismatch.');
assert(abs(Fz(4) - expected_rear_Fz/2) < tol,  'Test 1 Failed: Fz_RR mismatch.');
fprintf('PASSED (sum(Fz) = %.2f N, front/rear = %.2f/%.2f N)\n', ...
    sum(Fz), Fz_axles.front, Fz_axles.rear);

%% Test 2: Force Transformation to Vehicle Body Frame
fprintf('Test 2: Force transformation with front steering... ');
delta_2 = 0.15; % rad (~8.59 deg)
Fx_w_2 = [100; 100; 100; 100]; % 100 N longitudinal per wheel
Fy_w_2 = [ 20;  20;   0;   0]; % 20 N lateral on front wheels

[~, dyn_2] = fourWheelDynamics(2.0, 0.0, 0.0, delta_2, Fx_w_2, Fy_w_2, params);

% Analytical expected body forces for front wheels:
% Fx_body =  cos(delta)*Fx_w - sin(delta)*Fy_w
% Fy_body =  sin(delta)*Fx_w + cos(delta)*Fy_w
exp_Fx_body_front = cos(delta_2) * 100.0 - sin(delta_2) * 20.0;
exp_Fy_body_front = sin(delta_2) * 100.0 + cos(delta_2) * 20.0;

assert(abs(dyn_2.Fx_body(1) - exp_Fx_body_front) < tol, 'Test 2 Failed: FL Fx_body mismatch.');
assert(abs(dyn_2.Fx_body(2) - exp_Fx_body_front) < tol, 'Test 2 Failed: FR Fx_body mismatch.');
assert(abs(dyn_2.Fy_body(1) - exp_Fy_body_front) < tol, 'Test 2 Failed: FL Fy_body mismatch.');
assert(abs(dyn_2.Fy_body(2) - exp_Fy_body_front) < tol, 'Test 2 Failed: FR Fy_body mismatch.');

% Rear wheels (unsteered):
assert(abs(dyn_2.Fx_body(3) - 100.0) < tol, 'Test 2 Failed: RL Fx_body mismatch.');
assert(abs(dyn_2.Fx_body(4) - 100.0) < tol, 'Test 2 Failed: RR Fx_body mismatch.');
assert(abs(dyn_2.Fy_body(3) - 0.0) < tol,   'Test 2 Failed: RL Fy_body mismatch.');
assert(abs(dyn_2.Fy_body(4) - 0.0) < tol,   'Test 2 Failed: RR Fy_body mismatch.');
fprintf('PASSED (front Fx_body=%.4f N, Fy_body=%.4f N)\n', ...
    exp_Fx_body_front, exp_Fy_body_front);

%% Test 3: Individual Wheel Yaw Moment Sanity Check
fprintf('Test 3: Individual wheel yaw moment calculation... ');
delta_3 = 0.0;
Fx_w_3 = [ 50; -50;  50; -50];
Fy_w_3 = [ 10;  10; -10; -10];

[~, dyn_3] = fourWheelDynamics(1.0, 0.0, 0.0, delta_3, Fx_w_3, Fy_w_3, params);

% Analytical moment per wheel: Mz_i = x_i * Fy_i - y_i * Fx_i
% FL (+0.40, +0.30): Mz_FL = (+0.40)*(10)  - (+0.30)*( 50) = 4.0 - 15.0 = -11.0 N*m
% FR (+0.40, -0.30): Mz_FR = (+0.40)*(10)  - (-0.30)*(-50) = 4.0 - 15.0 = -11.0 N*m
% RL (-0.40, +0.30): Mz_RL = (-0.40)*(-10) - (+0.30)*( 50) = 4.0 - 15.0 = -11.0 N*m
% RR (-0.40, -0.30): Mz_RR = (-0.40)*(-10) - (-0.30)*(-50) = 4.0 - 15.0 = -11.0 N*m
expected_Mz_i = [-11.0; -11.0; -11.0; -11.0];
expected_Mz_total = -44.0;

assert(all(abs(dyn_3.Mz_i - expected_Mz_i) < tol), 'Test 3 Failed: Individual Mz_i mismatch.');
assert(abs(dyn_3.Mz_total - expected_Mz_total) < tol, 'Test 3 Failed: Total Mz mismatch.');
fprintf('PASSED (individual Mz_i match analytical moment formula exactly)\n');

%% Test 4: Pure Symmetric Forward Tractive Acceleration
fprintf('Test 4: Pure symmetric tractive acceleration... ');
delta_4 = 0.0;
Fx_w_4 = [50; 50; 50; 50]; % 200 N total forward force
Fy_w_4 = [ 0;  0;  0;  0];

state_deriv_4 = fourWheelDynamics(1.0, 0.0, 0.0, delta_4, Fx_w_4, Fy_w_4, params);

expected_dot_vx_4 = 200.0 / params.m; % 200 / 60 = 3.3333 m/s^2

assert(abs(state_deriv_4(1) - expected_dot_vx_4) < tol, 'Test 4 Failed: dot_vx mismatch.');
assert(abs(state_deriv_4(2)) < tol, 'Test 4 Failed: dot_vy should be 0.');
assert(abs(state_deriv_4(3)) < tol, 'Test 4 Failed: dot_r should be 0.');
fprintf('PASSED (dot(vx) = %.4f m/s^2, dot(vy) = 0, dot(r) = 0)\n', state_deriv_4(1));

%% Test 5: Differential Longitudinal Traction (Pure Yaw Acceleration)
fprintf('Test 5: Differential longitudinal forces (pure yaw moment)... ');
delta_5 = 0.0;
Fx_w_5 = [-50; +50; -50; +50]; % Left wheels braking (-50), right wheels driving (+50)
Fy_w_5 = [  0;   0;   0;   0];

[state_deriv_5, dyn_5] = fourWheelDynamics(0.0, 0.0, 0.0, delta_5, Fx_w_5, Fy_w_5, params);

% Yaw moments:
% FL: - (+0.30) * (-50) = +15 N*m
% FR: - (-0.30) * (+50) = +15 N*m
% RL: - (+0.30) * (-50) = +15 N*m
% RR: - (-0.30) * (+50) = +15 N*m
% Total Mz = +60 N*m (CCW yaw)
expected_Mz_5 = 60.0;
expected_dot_r_5 = expected_Mz_5 / params.Iz; % 60 / 5.0 = 12.0 rad/s^2

assert(abs(dyn_5.Fx_total) < tol, 'Test 5 Failed: Fx_total should be 0.');
assert(abs(dyn_5.Fy_total) < tol, 'Test 5 Failed: Fy_total should be 0.');
assert(abs(dyn_5.Mz_total - expected_Mz_5) < tol, 'Test 5 Failed: Total Mz mismatch.');
assert(abs(state_deriv_5(1)) < tol, 'Test 5 Failed: dot_vx should be 0.');
assert(abs(state_deriv_5(2)) < tol, 'Test 5 Failed: dot_vy should be 0.');
assert(abs(state_deriv_5(3) - expected_dot_r_5) < tol, 'Test 5 Failed: dot_r mismatch.');
fprintf('PASSED (Mz = +%.1f N*m, dot(r) = +%.2f rad/s^2)\n', expected_Mz_5, state_deriv_5(3));

%% Test 6: Kinematic Acceleration Coupling (Centripetal Terms)
fprintf('Test 6: Kinematic acceleration coupling terms (vy*r and -vx*r)... ');
vx_6 = 5.0; vy_6 = 1.0; r_6 = 2.0;
delta_6 = 0.0;
Fx_w_6 = zeros(4, 1);
Fy_w_6 = zeros(4, 1);

state_deriv_6 = fourWheelDynamics(vx_6, vy_6, r_6, delta_6, Fx_w_6, Fy_w_6, params);

% dot(vx) = (0/m) + vy*r = 1.0 * 2.0 = +2.0 m/s^2
% dot(vy) = (0/m) - vx*r = -5.0 * 2.0 = -10.0 m/s^2
% dot(r)  = 0 / Iz = 0.0
assert(abs(state_deriv_6(1) - 2.0) < tol,   'Test 6 Failed: Coupling dot_vx mismatch.');
assert(abs(state_deriv_6(2) - (-10.0)) < tol, 'Test 6 Failed: Coupling dot_vy mismatch.');
assert(abs(state_deriv_6(3) - 0.0) < tol,   'Test 6 Failed: Coupling dot_r mismatch.');
fprintf('PASSED (dot(vx) = +2.00 m/s^2, dot(vy) = -10.00 m/s^2)\n');

%% Test 7: Standstill Equilibrium (vx=0, vy=0, r=0, zero forces)
fprintf('Test 7: Standstill equilibrium (vx=vy=r=0, zero forces)... ');
vx_7 = 0.0; vy_7 = 0.0; r_7 = 0.0;
delta_7 = 0.0;
Fx_w_7 = zeros(4, 1);
Fy_w_7 = zeros(4, 1);

state_deriv_7 = fourWheelDynamics(vx_7, vy_7, r_7, delta_7, Fx_w_7, Fy_w_7, params);

assert(abs(state_deriv_7(1)) < tol, 'Test 7 Failed: dot_vx must be 0 at standstill.');
assert(abs(state_deriv_7(2)) < tol, 'Test 7 Failed: dot_vy must be 0 at standstill.');
assert(abs(state_deriv_7(3)) < tol, 'Test 7 Failed: dot_r must be 0 at standstill.');
fprintf('PASSED (dvx = 0, dvy = 0, dr = 0)\n');

%% Test 8: Constant Straight-Line State (vx>0, vy=0, r=0, zero forces)
fprintf('Test 8: Constant straight-line state (vx=3.0, vy=0, r=0, zero forces)... ');
vx_8 = 3.0; vy_8 = 0.0; r_8 = 0.0;
delta_8 = 0.0;
Fx_w_8 = zeros(4, 1);
Fy_w_8 = zeros(4, 1);

state_deriv_8 = fourWheelDynamics(vx_8, vy_8, r_8, delta_8, Fx_w_8, Fy_w_8, params);

assert(abs(state_deriv_8(1)) < tol, 'Test 8 Failed: dot_vx must be 0 for unaccelerated motion.');
assert(abs(state_deriv_8(2)) < tol, 'Test 8 Failed: dot_vy must be 0 for unaccelerated motion.');
assert(abs(state_deriv_8(3)) < tol, 'Test 8 Failed: dot_r must be 0 for unaccelerated motion.');
fprintf('PASSED (dvx = 0, dvy = 0, dr = 0)\n');

%% Summary
fprintf('\n====================================================\n');
fprintf('  All 8 Dynamics Verification Tests PASSED!\n');
fprintf('====================================================\n');
