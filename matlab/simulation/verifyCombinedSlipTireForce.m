% verifyCombinedSlipTireForce.m
% Verification script for Phase 3C: Reduced Combined-Slip Magic Formula Tire Model

clc;
clear;
close all;

fprintf('============================================================\n');
fprintf('  Running Verification: combinedSlipTireForce.m\n');
fprintf('============================================================\n\n');

%% Common Parameters
Fz = 294.3; % N (Nominal static normal load per wheel)
pure_params = struct('Bx', 10.0, 'Cx', 1.90, 'Dx', 1.00, 'Ex', 0.97, ...
                     'By', 10.0, 'Cy', 1.30, 'Dy', 1.00, 'Ey', -1.00);
combined_params = struct('Bx_alpha', 10.0, 'Cx_alpha', 1.00, 'Ex_alpha', -1.00, 'SHx_alpha', 0.0, ...
                         'By_kappa', 10.0, 'Cy_kappa', 1.00, 'Ey_kappa', -1.00, 'SHy_kappa', 0.0);

all_tests_passed = true;

%% Test 1: Zero slip equilibrium
kappa = 0; alpha = 0;
[Fx, Fy, Gxa, Gyk] = combinedSlipTireForce(kappa, alpha, Fz, pure_params, combined_params);
if abs(Fx) < 1e-10 && abs(Fy) < 1e-10 && abs(Gxa - 1) < 1e-10 && abs(Gyk - 1) < 1e-10
    fprintf('Test 1: Zero slip equilibrium... PASSED (Fx = 0, Fy = 0, Gxa = 1, Gyk = 1)\n');
else
    fprintf('Test 1: FAILED\n');
    all_tests_passed = false;
end

%% Test 2: Pure longitudinal recovery
kappa = 0.15; alpha = 0;
[Fx, Fy, Gxa, Gyk] = combinedSlipTireForce(kappa, alpha, Fz, pure_params, combined_params);
Fx0 = magicFormulaLongitudinal(kappa, Fz, pure_params.Bx, pure_params.Cx, pure_params.Dx, pure_params.Ex);
if abs(Gxa - 1) < 1e-10 && abs(Fx - Fx0) < 1e-10
    fprintf('Test 2: Pure longitudinal recovery... PASSED (alpha = 0 -> Gxa = 1, Fx = Fx0)\n');
else
    fprintf('Test 2: FAILED\n');
    all_tests_passed = false;
end

%% Test 3: Pure lateral recovery
kappa = 0; alpha = 0.15;
[Fx, Fy, Gxa, Gyk] = combinedSlipTireForce(kappa, alpha, Fz, pure_params, combined_params);
Fy0 = magicFormulaLateral(alpha, Fz, pure_params.By, pure_params.Cy, pure_params.Dy, pure_params.Ey);
if abs(Gyk - 1) < 1e-10 && abs(Fy - Fy0) < 1e-10
    fprintf('Test 3: Pure lateral recovery... PASSED (kappa = 0 -> Gyk = 1, Fy = Fy0)\n');
else
    fprintf('Test 3: FAILED\n');
    all_tests_passed = false;
end

%% Test 4: Combined braking + cornering
kappa = -0.15; alpha = 0.15;
[Fx, Fy, Gxa, Gyk] = combinedSlipTireForce(kappa, alpha, Fz, pure_params, combined_params);
Fx0 = magicFormulaLongitudinal(kappa, Fz, pure_params.Bx, pure_params.Cx, pure_params.Dx, pure_params.Ex);
Fy0 = magicFormulaLateral(alpha, Fz, pure_params.By, pure_params.Cy, pure_params.Dy, pure_params.Ey);
if abs(Fx) < abs(Fx0) && abs(Fy) < abs(Fy0) && Gxa < 1.0 && Gyk < 1.0
    fprintf('Test 4: Combined braking + cornering... PASSED (|Fx| < |Fx0|, |Fy| < |Fy0|)\n');
else
    fprintf('Test 4: FAILED\n');
    all_tests_passed = false;
end

%% Test 5: Combined driving + cornering
kappa = 0.15; alpha = 0.15;
[Fx, Fy, Gxa, Gyk] = combinedSlipTireForce(kappa, alpha, Fz, pure_params, combined_params);
Fx0 = magicFormulaLongitudinal(kappa, Fz, pure_params.Bx, pure_params.Cx, pure_params.Dx, pure_params.Ex);
Fy0 = magicFormulaLateral(alpha, Fz, pure_params.By, pure_params.Cy, pure_params.Dy, pure_params.Ey);
if abs(Fx) < abs(Fx0) && abs(Fy) < abs(Fy0) && Gxa < 1.0 && Gyk < 1.0
    fprintf('Test 5: Combined driving + cornering... PASSED (|Fx| < |Fx0|, |Fy| < |Fy0|)\n');
else
    fprintf('Test 5: FAILED\n');
    all_tests_passed = false;
end

%% Test 6: Sign consistency across all 4 quadrants
k_vals = [0.15, -0.15, -0.15, 0.15];
a_vals = [0.15, 0.15, -0.15, -0.15];
Fx_signs = [1, -1, -1, 1];
Fy_signs = [-1, -1, 1, 1];
signs_ok = true;
for i = 1:4
    [Fx, Fy, ~, ~] = combinedSlipTireForce(k_vals(i), a_vals(i), Fz, pure_params, combined_params);
    if sign(Fx) ~= Fx_signs(i) || sign(Fy) ~= Fy_signs(i)
        signs_ok = false;
    end
end
if signs_ok
    fprintf('Test 6: Four quadrant sign consistency... PASSED\n');
else
    fprintf('Test 6: FAILED\n');
    all_tests_passed = false;
end

%% Test 7: Longitudinal odd symmetry in kappa
[Fx_pos, ~, ~, ~] = combinedSlipTireForce(0.15, 0.10, Fz, pure_params, combined_params);
[Fx_neg, ~, ~, ~] = combinedSlipTireForce(-0.15, 0.10, Fz, pure_params, combined_params);
if abs(Fx_pos - (-Fx_neg)) < 1e-10
    fprintf('Test 7: Fx odd symmetry in kappa... PASSED (Fx(-kappa, alpha) == -Fx(kappa, alpha))\n');
else
    fprintf('Test 7: FAILED\n');
    all_tests_passed = false;
end

%% Test 8: Lateral odd symmetry in alpha
[~, Fy_pos, ~, ~] = combinedSlipTireForce(0.10, 0.15, Fz, pure_params, combined_params);
[~, Fy_neg, ~, ~] = combinedSlipTireForce(0.10, -0.15, Fz, pure_params, combined_params);
if abs(Fy_pos - (-Fy_neg)) < 1e-10
    fprintf('Test 8: Fy odd symmetry in alpha... PASSED (Fy(kappa, -alpha) == -Fy(kappa, alpha))\n');
else
    fprintf('Test 8: FAILED\n');
    all_tests_passed = false;
end

%% Test 9: Longitudinal even symmetry in alpha for SH = 0
[Fx_posA, ~, ~, ~] = combinedSlipTireForce(0.15, 0.10, Fz, pure_params, combined_params);
[Fx_negA, ~, ~, ~] = combinedSlipTireForce(0.15, -0.10, Fz, pure_params, combined_params);
if abs(Fx_posA - Fx_negA) < 1e-10
    fprintf('Test 9: Fx even symmetry in alpha (SH=0)... PASSED (Fx(kappa, -alpha) == Fx(kappa, alpha))\n');
else
    fprintf('Test 9: FAILED\n');
    all_tests_passed = false;
end

%% Test 10: Lateral even symmetry in kappa for SH = 0
[~, Fy_posK, ~, ~] = combinedSlipTireForce(0.10, 0.15, Fz, pure_params, combined_params);
[~, Fy_negK, ~, ~] = combinedSlipTireForce(-0.10, 0.15, Fz, pure_params, combined_params);
if abs(Fy_posK - Fy_negK) < 1e-10
    fprintf('Test 10: Fy even symmetry in kappa (SH=0)... PASSED (Fy(-kappa, alpha) == Fy(kappa, alpha))\n');
else
    fprintf('Test 10: FAILED\n');
    all_tests_passed = false;
end

%% Test 11: Vectorized evaluation vs scalar
kappa_vec = linspace(-0.2, 0.2, 5)';
alpha_vec = linspace(-0.2, 0.2, 5)';
Fz_vec = Fz * ones(5, 1);
[Fx_vec, Fy_vec, ~, ~] = combinedSlipTireForce(kappa_vec, alpha_vec, Fz_vec, pure_params, combined_params);

max_diff = 0;
for i = 1:5
    [Fx_s, Fy_s, ~, ~] = combinedSlipTireForce(kappa_vec(i), alpha_vec(i), Fz_vec(i), pure_params, combined_params);
    max_diff = max(max_diff, abs(Fx_vec(i) - Fx_s) + abs(Fy_vec(i) - Fy_s));
end

if max_diff < 1e-10
    fprintf('Test 11: Vectorized evaluation... PASSED\n');
else
    fprintf('Test 11: FAILED\n');
    all_tests_passed = false;
end

%% Test 12: Dimension mismatch rejection
try
    combinedSlipTireForce(ones(3,1), ones(2,1), Fz, pure_params, combined_params);
    fprintf('Test 12: FAILED (did not throw error for dimension mismatch)\n');
    all_tests_passed = false;
catch ME
    if strcmp(ME.identifier, 'combinedSlipTireForce:dimensionMismatch')
        fprintf('Test 12: Dimension mismatch rejection... PASSED\n');
    else
        fprintf('Test 12: FAILED (wrong error identifier)\n');
        all_tests_passed = false;
    end
end

%% Test 13: Invalid Fz rejection
try
    combinedSlipTireForce(0.1, 0.1, 0, pure_params, combined_params);
    fprintf('Test 13: FAILED (did not throw error for Fz=0)\n');
    all_tests_passed = false;
catch ME
    if strcmp(ME.identifier, 'combinedSlipTireForce:invalidNormalLoad')
        fprintf('Test 13: Invalid normal load rejection... PASSED\n');
    else
        fprintf('Test 13: FAILED (wrong error identifier)\n');
        all_tests_passed = false;
    end
end

%% Test 14: Dense grid robustness
[k_grid, a_grid] = ndgrid(linspace(-1, 1, 100), linspace(-0.5, 0.5, 100));
[Fx_grid, Fy_grid, Gxa_grid, Gyk_grid] = combinedSlipTireForce(k_grid, a_grid, Fz, pure_params, combined_params);
if ~any(isnan(Fx_grid(:))) && ~any(isinf(Fx_grid(:))) && ...
   ~any(isnan(Fy_grid(:))) && ~any(isinf(Fy_grid(:))) && ...
   min(Gxa_grid(:)) > -1e-6 && min(Gyk_grid(:)) > -1e-6
    fprintf('Test 14: Dense grid robustness (no NaN/Inf, G >= 0)... PASSED\n');
else
    fprintf('Test 14: FAILED\n');
    all_tests_passed = false;
end

%% Test 15: direct combinedSlipWeighting non-zero shift unit test
SH_test = 0.05;
G_test_0   = combinedSlipWeighting(0,   10.0, 1.0, -1.0, SH_test);
G_test_pos = combinedSlipWeighting(0.1, 10.0, 1.0, -1.0, SH_test);
G_test_neg = combinedSlipWeighting(-0.1, 10.0, 1.0, -1.0, SH_test);

norm_ok    = abs(G_test_0 - 1.0) < 1e-10;
finite_ok  = isfinite(G_test_pos) && isfinite(G_test_neg);
not_even   = abs(G_test_pos - G_test_neg) > 1e-10; % shifted G is not even

if norm_ok && finite_ok && not_even
    fprintf('Test 15: combinedSlipWeighting with SH~=0... PASSED (G(0) = 1.0, finite, not even)\n');
else
    fprintf('Test 15: FAILED (norm=%d, finite=%d, not_even=%d)\n', norm_ok, finite_ok, not_even);
    all_tests_passed = false;
end

fprintf('\n');
if all_tests_passed
    fprintf('============================================================\n');
    fprintf('  All 15 combined-slip tire force tests PASSED successfully.\n');
    fprintf('============================================================\n');
else
    fprintf('============================================================\n');
    fprintf('  ONE OR MORE TESTS FAILED.\n');
    fprintf('============================================================\n');
end
