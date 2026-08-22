% verifyMagicFormulaLongitudinal.m
% Verification script for magicFormulaLongitudinal function.
%
% Tests:
%   1. Zero slip condition (kappa == 0 -> Fx == 0)
%   2. Positive driving slip (kappa > 0 -> Fx > 0)
%   3. Negative braking slip (kappa < 0 -> Fx < 0)
%   4. Vectorized evaluation against scalar evaluations
%   5. Normal-load scaling linearity (Fx proportional to Fz for constant parameters)
%   6. Zero and negative normal-load rejection (Fz <= 0)
%   7. Incompatible dimension rejection (kappa vs Fz size mismatch)
%   8. Small-slip slope consistency with analytical formula (K0 = B * C * D * Fz)

clear;
clc;

fprintf('============================================================\n');
fprintf('  Running Verification: magicFormulaLongitudinal.m\n');
fprintf('============================================================\n\n');

% Baseline representative model parameters (not measured experimental data)
Fz_base = 4000.0;   % Vertical tire normal load [N]
B = 10.0;           % Stiffness factor [-]
C = 1.9;            % Shape factor [-]
D = 1.0;            % Peak factor [-]
E = 0.97;           % Curvature factor [-]

tol = 1e-12;        % Numerical tolerance for equality checks

%% Test 1: Zero Slip Condition
fprintf('Test 1: Zero slip (kappa = 0 -> Fx = 0)... ');
kappa_0 = 0.0;
Fx_0 = magicFormulaLongitudinal(kappa_0, Fz_base, B, C, D, E);

assert(isfinite(Fx_0), 'Test 1 Failed: Output is not finite.');
assert(abs(Fx_0) < tol, 'Test 1 Failed: Fx at zero slip should be 0, got %e N.', Fx_0);
fprintf('PASSED (Fx = %.6e N)\n', Fx_0);

%% Test 2: Positive Driving Slip
fprintf('Test 2: Driving slip (kappa > 0 -> Fx > 0)... ');
kappa_drive = 0.15;
Fx_drive = magicFormulaLongitudinal(kappa_drive, Fz_base, B, C, D, E);

assert(isfinite(Fx_drive), 'Test 2 Failed: Output is not finite.');
assert(Fx_drive > 0, 'Test 2 Failed: Fx under positive slip should be positive, got %f N.', Fx_drive);
fprintf('PASSED (kappa = +%.2f -> Fx = +%.2f N)\n', kappa_drive, Fx_drive);

%% Test 3: Negative Braking Slip
fprintf('Test 3: Braking slip (kappa < 0 -> Fx < 0)... ');
kappa_brake = -0.15;
Fx_brake = magicFormulaLongitudinal(kappa_brake, Fz_base, B, C, D, E);

assert(isfinite(Fx_brake), 'Test 3 Failed: Output is not finite.');
assert(Fx_brake < 0, 'Test 3 Failed: Fx under negative slip should be negative, got %f N.', Fx_brake);
assert(abs(Fx_brake + Fx_drive) < tol, ...
    'Test 3 Failed: Function should be an odd function of kappa, asymmetry = %e N.', abs(Fx_brake + Fx_drive));
fprintf('PASSED (kappa = %.2f -> Fx = %.2f N, symmetric)\n', kappa_brake, Fx_brake);

%% Test 4: Vectorized Evaluation
fprintf('Test 4: Vectorized evaluation vs scalar evaluations... ');
kappa_vec = linspace(-1.0, 1.0, 201)'; % 201x1 vector from full braking to 100% spin
Fx_vec = magicFormulaLongitudinal(kappa_vec, Fz_base, B, C, D, E);

assert(isequal(size(Fx_vec), size(kappa_vec)), 'Test 4 Failed: Output dimension mismatch.');
assert(all(isfinite(Fx_vec)), 'Test 4 Failed: Non-finite values in vector evaluation.');

% Validate against scalar loop evaluation
Fx_scalar = zeros(size(kappa_vec));
for i = 1:length(kappa_vec)
    Fx_scalar(i) = magicFormulaLongitudinal(kappa_vec(i), Fz_base, B, C, D, E);
end
max_diff = max(abs(Fx_vec - Fx_scalar));
assert(max_diff < tol, 'Test 4 Failed: Vectorized results differ from scalar evaluation by %e N.', max_diff);
fprintf('PASSED (201 points evaluated, max diff = %.2e N)\n', max_diff);

%% Test 5: Normal-Load Scaling
fprintf('Test 5: Normal-load linear scaling... ');
Fz_1 = 3000.0;
Fz_2 = 6000.0;
kappa_test = 0.10;

Fx_1 = magicFormulaLongitudinal(kappa_test, Fz_1, B, C, D, E);
Fx_2 = magicFormulaLongitudinal(kappa_test, Fz_2, B, C, D, E);

expected_ratio = Fz_2 / Fz_1; % 2.0
actual_ratio = Fx_2 / Fx_1;

assert(abs(actual_ratio - expected_ratio) < tol, ...
    'Test 5 Failed: Force ratio %f does not match load ratio %f.', actual_ratio, expected_ratio);
fprintf('PASSED (Fx scales exactly with Fz: ratio = %.4f)\n', actual_ratio);

%% Test 6: Zero and Negative Normal-Load Rejection
fprintf('Test 6: Invalid load rejection (Fz <= 0)... ');

% Zero load test
threwErrorZero = false;
try
    magicFormulaLongitudinal(0.1, 0.0, B, C, D, E);
catch
    threwErrorZero = true;
end
assert(threwErrorZero, 'Test 6 Failed: Function accepted Fz = 0.');

% Negative load test
threwErrorNeg = false;
try
    magicFormulaLongitudinal(0.1, -1000.0, B, C, D, E);
catch
    threwErrorNeg = true;
end
assert(threwErrorNeg, 'Test 6 Failed: Function accepted negative Fz.');
fprintf('PASSED (Fz = 0 and Fz < 0 successfully rejected)\n');

%% Test 7: Incompatible Dimension Rejection
fprintf('Test 7: Incompatible dimension rejection... ');
kappa_incompat = [0.1, 0.2, 0.3];      % 1x3 row vector
Fz_incompat = [3000; 4000; 5000; 6000]; % 4x1 column vector

threwErrorDim = false;
try
    magicFormulaLongitudinal(kappa_incompat, Fz_incompat, B, C, D, E);
catch ME
    threwErrorDim = true;
    assert(strcmp(ME.identifier, 'magicFormulaLongitudinal:dimensionMismatch'), ...
        'Test 7 Failed: Expected dimensionMismatch error, got %s', ME.identifier);
end
assert(threwErrorDim, 'Test 7 Failed: Function did not reject incompatible dimensions.');
fprintf('PASSED (dimension mismatch correctly rejected)\n');

%% Test 8: Small-Slip Slope (Initial Stiffness K0)
fprintf('Test 8: Small-slip initial stiffness consistency... ');
% Theoretical analytical slope at kappa = 0: K0 = B * C * D * Fz
K0_analytical = B * C * D * Fz_base; % 10 * 1.9 * 1.0 * 4000 = 76000 N/slip

% Numerical forward difference at small delta_kappa
d_kappa = 1e-5;
Fx_small = magicFormulaLongitudinal(d_kappa, Fz_base, B, C, D, E);
K0_numerical = Fx_small / d_kappa;

rel_error = abs(K0_numerical - K0_analytical) / K0_analytical;
rel_tol = 1e-4; % 0.01% tolerance for finite difference approximation

assert(rel_error < rel_tol, ...
    'Test 8 Failed: Numerical slope %f differs from analytical K0 %f (rel error = %e).', ...
    K0_numerical, K0_analytical, rel_error);
fprintf('PASSED (Analytical K0 = %.1f N, Numerical = %.1f N, Rel error = %.2e%%)\n', ...
    K0_analytical, K0_numerical, rel_error * 100);

%% Summary
fprintf('\n============================================================\n');
fprintf('  All 8 verification tests PASSED successfully.\n');
fprintf('============================================================\n');
