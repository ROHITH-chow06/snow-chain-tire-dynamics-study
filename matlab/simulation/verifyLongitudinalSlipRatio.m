% verifyLongitudinalSlipRatio.m
% Verification script for longitudinalSlipRatio function.
%
% Tests:
%   1. Pure rolling condition (R * omega == Vx -> kappa == 0)
%   2. Positive driving slip (R * omega > Vx -> kappa > 0)
%   3. Negative braking slip (R * omega < Vx -> kappa < 0)
%   4. Vectorized evaluation across multiple operating points
%   5. Near-zero vehicle speed regularization (no NaN or Inf)
%   6. Locked wheel braking test (omega == 0 -> kappa == -1 at normal speed)

clear;
clc;

fprintf('====================================================\n');
fprintf('  Running Verification: longitudinalSlipRatio.m\n');
fprintf('====================================================\n\n');

% Physical test parameters (representative passenger car / light vehicle)
R = 0.30;       % Wheel rolling radius [m]
tol = 1e-12;    % Numerical tolerance for equality checks

%% Test 1: Pure Rolling Condition
fprintf('Test 1: Pure rolling (R * omega == Vx)... ');
Vx_1 = 20.0;            % Vehicle speed [m/s] (~72 km/h)
omega_1 = Vx_1 / R;     % Angular velocity matching ground speed [rad/s]

kappa_1 = longitudinalSlipRatio(R, omega_1, Vx_1);

assert(isfinite(kappa_1), 'Test 1 Failed: Result is not finite.');
assert(abs(kappa_1) < tol, 'Test 1 Failed: Pure rolling slip should be 0, got %e.', kappa_1);
fprintf('PASSED (kappa = %.6e)\n', kappa_1);

%% Test 2: Positive Driving Slip (Acceleration)
fprintf('Test 2: Driving slip (R * omega > Vx)... ');
Vx_2 = 15.0;            % Vehicle speed [m/s]
omega_2 = 60.0;         % Wheel spinning faster: R * omega = 18 m/s > 15 m/s

kappa_2 = longitudinalSlipRatio(R, omega_2, Vx_2);
expected_kappa_2 = (R * omega_2 - Vx_2) / Vx_2; % (18 - 15) / 15 = 0.20

assert(isfinite(kappa_2), 'Test 2 Failed: Result is not finite.');
assert(kappa_2 > 0, 'Test 2 Failed: Driving slip should be strictly positive.');
assert(abs(kappa_2 - expected_kappa_2) < tol, ...
    'Test 2 Failed: Expected %f, got %f.', expected_kappa_2, kappa_2);
fprintf('PASSED (kappa = +%.4f, expected = +%.4f)\n', kappa_2, expected_kappa_2);

%% Test 3: Negative Braking Slip (Deceleration)
fprintf('Test 3: Braking slip (R * omega < Vx)... ');
Vx_3 = 25.0;            % Vehicle speed [m/s]
omega_3 = 70.0;         % Wheel spinning slower: R * omega = 21 m/s < 25 m/s

kappa_3 = longitudinalSlipRatio(R, omega_3, Vx_3);
expected_kappa_3 = (R * omega_3 - Vx_3) / Vx_3; % (21 - 25) / 25 = -0.16

assert(isfinite(kappa_3), 'Test 3 Failed: Result is not finite.');
assert(kappa_3 < 0, 'Test 3 Failed: Braking slip should be strictly negative.');
assert(abs(kappa_3 - expected_kappa_3) < tol, ...
    'Test 3 Failed: Expected %f, got %f.', expected_kappa_3, kappa_3);
fprintf('PASSED (kappa = %.4f, expected = %.4f)\n', kappa_3, expected_kappa_3);

%% Test 4: Locked Wheel Braking (omega = 0)
fprintf('Test 4: Locked wheel braking (omega = 0, Vx > 0)... ');
Vx_4 = 20.0;
omega_4 = 0.0;

kappa_4 = longitudinalSlipRatio(R, omega_4, Vx_4);
expected_kappa_4 = -1.0;

assert(abs(kappa_4 - expected_kappa_4) < tol, ...
    'Test 4 Failed: Locked wheel slip should be -1.0, got %f.', kappa_4);
fprintf('PASSED (kappa = %.4f)\n', kappa_4);

%% Test 5: Vectorized Input Handling
fprintf('Test 5: Vectorized evaluation... ');
Vx_vec = [10; 15; 20; 25];              % 4x1 vector [m/s]
omega_vec = [30; 55; 66.666666666667; 75]; % Varying angular speeds [rad/s]

kappa_vec = longitudinalSlipRatio(R, omega_vec, Vx_vec);

assert(isequal(size(kappa_vec), size(Vx_vec)), ...
    'Test 5 Failed: Output dimension mismatch.');
assert(all(isfinite(kappa_vec)), 'Test 5 Failed: Non-finite elements found in vector output.');

expected_vec = (R .* omega_vec - Vx_vec) ./ Vx_vec;
assert(max(abs(kappa_vec - expected_vec)) < 1e-10, ...
    'Test 5 Failed: Vector calculation does not match analytical expectation.');
fprintf('PASSED (4 points evaluated successfully)\n');

%% Test 6: Near-Zero Vehicle Speed and Standstill Regularization
fprintf('Test 6: Near-zero speed and standstill robustness... ');
eps_custom = 0.05;

% Standstill with zero angular velocity
kappa_zero = longitudinalSlipRatio(R, 0, 0, eps_custom);
assert(isfinite(kappa_zero), 'Test 6 Failed: Standstill produced non-finite output.');
assert(abs(kappa_zero) < tol, 'Test 6 Failed: Standstill with omega=0 should yield 0, got %f.', kappa_zero);

% Near-zero forward speed with small wheel spin
Vx_near_zero = 1e-6;
omega_spin = 1.0; % R * omega = 0.3 m/s
kappa_near_zero = longitudinalSlipRatio(R, omega_spin, Vx_near_zero, eps_custom);

assert(isfinite(kappa_near_zero), 'Test 6 Failed: Near-zero speed produced non-finite output.');
assert(~isnan(kappa_near_zero) && ~isinf(kappa_near_zero), ...
    'Test 6 Failed: Output is NaN or Inf.');

% Expected value regularized by eps_custom (since |Vx| < eps_custom, denom = eps_custom)
expected_near_zero = (R * omega_spin - Vx_near_zero) / eps_custom;
assert(abs(kappa_near_zero - expected_near_zero) < tol, ...
    'Test 6 Failed: Regularization calculation mismatch.');
fprintf('PASSED (standstill & eps-regularization verified)\n');

%% Summary
fprintf('\n====================================================\n');
fprintf('  All 6 verification tests PASSED successfully.\n');
fprintf('====================================================\n');
