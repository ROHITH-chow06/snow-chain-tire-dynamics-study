% verifyMagicFormulaLateral.m
% Verification script for magicFormulaLateral function.
%
% Tests:
%   1. Zero slip angle (alpha == 0 -> Fy == 0)
%   2. Positive velocity angle (alpha > 0 -> Fy < 0)
%   3. Negative velocity angle (alpha < 0 -> Fy > 0)
%   4. Odd symmetry (Fy(-alpha) == -Fy(alpha))
%   5. Vectorized evaluation against scalar evaluations
%   6. Normal-load scaling linearity (Fy proportional to Fz)
%   7. Zero and negative normal-load rejection (Fz <= 0)
%   8. Incompatible dimension rejection (alpha vs Fz size mismatch)
%   9. Small-slip initial cornering stiffness (dFy/dalpha = -By * Cy * Dy * Fz)
%   10. Peak-force sanity across slip-angle sweep

clear;
clc;

fprintf('============================================================\n');
fprintf('  Running Verification: magicFormulaLateral.m\n');
fprintf('============================================================\n\n');

% Baseline representative model parameters (not measured experimental data)
Fz_base = 4000.0;   % Vertical tire normal load [N]
By = 10.0;          % Lateral stiffness factor [-]
Cy = 1.30;          % Lateral shape factor [-]
Dy = 1.00;          % Lateral peak factor [-]
Ey = -1.00;         % Lateral curvature factor [-]

tol = 1e-12;        % Numerical tolerance for equality checks

%% Test 1: Zero Slip Angle Condition
fprintf('Test 1: Zero slip angle (alpha = 0 -> Fy = 0)... ');
alpha_0 = 0.0;
Fy_0 = magicFormulaLateral(alpha_0, Fz_base, By, Cy, Dy, Ey);

assert(isfinite(Fy_0), 'Test 1 Failed: Output is not finite.');
assert(abs(Fy_0) < tol, 'Test 1 Failed: Fy at zero slip angle should be 0, got %e N.', Fy_0);
fprintf('PASSED (Fy = %.6e N)\n', Fy_0);

%% Test 2: Positive Velocity Angle (alpha > 0 -> Fy < 0)
fprintf('Test 2: Positive velocity angle (alpha > 0 -> Fy < 0)... ');
alpha_pos = 0.10; % +0.10 rad (~5.73 deg)
Fy_pos = magicFormulaLateral(alpha_pos, Fz_base, By, Cy, Dy, Ey);

assert(isfinite(Fy_pos), 'Test 2 Failed: Output is not finite.');
assert(Fy_pos < 0, 'Test 2 Failed: Fy under positive velocity angle should be negative, got %f N.', Fy_pos);
fprintf('PASSED (alpha = +%.2f rad -> Fy = %.2f N)\n', alpha_pos, Fy_pos);

%% Test 3: Negative Velocity Angle (alpha < 0 -> Fy > 0)
fprintf('Test 3: Negative velocity angle (alpha < 0 -> Fy > 0)... ');
alpha_neg = -0.10; % -0.10 rad (~ -5.73 deg)
Fy_neg = magicFormulaLateral(alpha_neg, Fz_base, By, Cy, Dy, Ey);

assert(isfinite(Fy_neg), 'Test 3 Failed: Output is not finite.');
assert(Fy_neg > 0, 'Test 3 Failed: Fy under negative velocity angle should be positive, got %f N.', Fy_neg);
fprintf('PASSED (alpha = %.2f rad -> Fy = +%.2f N)\n', alpha_neg, Fy_neg);

%% Test 4: Odd Symmetry
fprintf('Test 4: Odd symmetry (Fy(-alpha) == -Fy(alpha))... ');
assert(abs(Fy_neg + Fy_pos) < tol, ...
    'Test 4 Failed: Function should exhibit exact odd symmetry, sum = %e N.', abs(Fy_neg + Fy_pos));
fprintf('PASSED (exact odd symmetry verified)\n');

%% Test 5: Vectorized Evaluation vs Scalar Evaluations
fprintf('Test 5: Vectorized evaluation vs scalar evaluations... ');
alpha_vec = linspace(-0.5, 0.5, 201)'; % 201x1 vector from -0.5 rad to +0.5 rad (~ +/-28.6 deg)
Fy_vec = magicFormulaLateral(alpha_vec, Fz_base, By, Cy, Dy, Ey);

assert(isequal(size(Fy_vec), size(alpha_vec)), 'Test 5 Failed: Output dimension mismatch.');
assert(all(isfinite(Fy_vec)), 'Test 5 Failed: Non-finite values in vector evaluation.');

% Validate against independent scalar loop evaluations
Fy_scalar = zeros(size(alpha_vec));
for i = 1:length(alpha_vec)
    Fy_scalar(i) = magicFormulaLateral(alpha_vec(i), Fz_base, By, Cy, Dy, Ey);
end
max_diff = max(abs(Fy_vec - Fy_scalar));
assert(max_diff < tol, 'Test 5 Failed: Vectorized results differ from scalar evaluation by %e N.', max_diff);
fprintf('PASSED (201 points evaluated, max diff = %.2e N)\n', max_diff);

%% Test 6: Normal-Load Linear Scaling
fprintf('Test 6: Normal-load linear scaling... ');
Fz_1 = 3000.0;
Fz_2 = 6000.0;
alpha_test = 0.08;

Fy_1 = magicFormulaLateral(alpha_test, Fz_1, By, Cy, Dy, Ey);
Fy_2 = magicFormulaLateral(alpha_test, Fz_2, By, Cy, Dy, Ey);

expected_ratio = Fz_2 / Fz_1; % 2.0
actual_ratio = Fy_2 / Fy_1;

assert(abs(actual_ratio - expected_ratio) < tol, ...
    'Test 6 Failed: Force ratio %f does not match load ratio %f.', actual_ratio, expected_ratio);
fprintf('PASSED (Fy scales exactly with Fz: ratio = %.4f)\n', actual_ratio);

%% Test 7: Zero and Negative Normal-Load Rejection
fprintf('Test 7: Invalid load rejection (Fz <= 0)... ');

% Zero load test
threwErrorZero = false;
try
    magicFormulaLateral(0.1, 0.0, By, Cy, Dy, Ey);
catch
    threwErrorZero = true;
end
assert(threwErrorZero, 'Test 7 Failed: Function accepted Fz = 0.');

% Negative load test
threwErrorNeg = false;
try
    magicFormulaLateral(0.1, -1000.0, By, Cy, Dy, Ey);
catch
    threwErrorNeg = true;
end
assert(threwErrorNeg, 'Test 7 Failed: Function accepted negative Fz.');
fprintf('PASSED (Fz = 0 and Fz < 0 successfully rejected)\n');

%% Test 8: Incompatible Dimension Rejection
fprintf('Test 8: Incompatible dimension rejection... ');
alpha_incompat = [0.05, 0.10, 0.15];      % 1x3 row vector
Fz_incompat = [3000; 4000; 5000; 6000];   % 4x1 column vector

threwErrorDim = false;
try
    magicFormulaLateral(alpha_incompat, Fz_incompat, By, Cy, Dy, Ey);
catch ME
    threwErrorDim = true;
    assert(strcmp(ME.identifier, 'magicFormulaLateral:dimensionMismatch'), ...
        'Test 8 Failed: Expected dimensionMismatch error, got %s', ME.identifier);
end
assert(threwErrorDim, 'Test 8 Failed: Function did not reject incompatible dimensions.');
fprintf('PASSED (dimension mismatch correctly rejected)\n');

%% Test 9: Initial Cornering Stiffness Consistency
fprintf('Test 9: Small-slip initial cornering stiffness (dFy/dalpha)... ');
% Theoretical analytical slope at alpha = 0: dFy/dalpha = -By * Cy * Dy * Fz
K_alpha_analytical = -By * Cy * Dy * Fz_base; % -10 * 1.30 * 1.00 * 4000 = -52000 N/rad

% Numerical central difference at small delta_alpha
d_alpha = 1e-5;
Fy_pos_d = magicFormulaLateral(+d_alpha, Fz_base, By, Cy, Dy, Ey);
Fy_neg_d = magicFormulaLateral(-d_alpha, Fz_base, By, Cy, Dy, Ey);
K_alpha_numerical = (Fy_pos_d - Fy_neg_d) / (2 * d_alpha);

rel_error = abs(K_alpha_numerical - K_alpha_analytical) / abs(K_alpha_analytical);
rel_tol = 1e-4; % 0.01% tolerance for central difference approximation

assert(rel_error < rel_tol, ...
    'Test 9 Failed: Numerical slope %f differs from analytical K_alpha %f (rel error = %e).', ...
    K_alpha_numerical, K_alpha_analytical, rel_error);
fprintf('PASSED (Analytical = %.1f N/rad, Numerical = %.1f N/rad, Rel error = %.2e%%)\n', ...
    K_alpha_analytical, K_alpha_numerical, rel_error * 100);

%% Test 10: Peak-Force Sanity and Saturation
fprintf('Test 10: Peak-force sanity and saturation... ');
alpha_sweep = linspace(-0.5, 0.5, 1001)';
Fy_sweep = magicFormulaLateral(alpha_sweep, Fz_base, By, Cy, Dy, Ey);

assert(all(isfinite(Fy_sweep)), 'Test 10 Failed: Non-finite values encountered in sweep.');

% Identify maximum force magnitude
[max_Fy_mag, max_idx] = max(abs(Fy_sweep));
alpha_peak = abs(alpha_sweep(max_idx));
expected_peak_mag = Dy * Fz_base; % 1.00 * 4000 = 4000 N

assert(abs(max_Fy_mag - expected_peak_mag) < 1.0, ...
    'Test 10 Failed: Peak force magnitude %f does not match expected peak %f.', ...
    max_Fy_mag, expected_peak_mag);
assert(alpha_peak > 0.05 && alpha_peak < 0.35, ...
    'Test 10 Failed: Peak slip angle %f rad (%.2f deg) outside expected range [0.05, 0.35] rad.', ...
    alpha_peak, rad2deg(alpha_peak));
fprintf('PASSED (Peak |Fy| = %.2f N at alpha = +/-%.4f rad [+/-%.2f deg])\n', ...
    max_Fy_mag, alpha_peak, rad2deg(alpha_peak));

%% Summary
fprintf('\n============================================================\n');
fprintf('  All 10 lateral Magic Formula tests PASSED successfully.\n');
fprintf('============================================================\n');
