% verifyDynamicNormalLoadDistribution.m
% Verification script for Phase 3D: Quasi-Static Four-Wheel Dynamic Load Transfer

clc;
clear;
close all;

fprintf('============================================================\n');
fprintf('  Running Verification: dynamicNormalLoadDistribution.m\n');
fprintf('============================================================\n\n');

params = roverParameters();
all_tests_passed = true;
tol = 1e-10;

%% Test 1: Zero acceleration (static equilibrium recovery)
ax = 0; ay = 0;
[Fz, diag1] = dynamicNormalLoadDistribution(params, ax, ay);
expected_Fz_static = [147.15; 147.15; 147.15; 147.15];

if max(abs(Fz - expected_Fz_static)) < tol && ...
   abs(diag1.DeltaFz_long) < tol && ...
   abs(diag1.DeltaFz_lat_front) < tol && ...
   abs(diag1.DeltaFz_lat_rear) < tol
    fprintf('Test 1: Zero acceleration (static equilibrium)... PASSED (all Fz = 147.1500 N)\n');
else
    fprintf('Test 1: FAILED\n');
    all_tests_passed = false;
end

%% Test 2: Positive longitudinal acceleration (ax > 0, ay = 0)
ax = 2.0; ay = 0.0;
[Fz2, diag2] = dynamicNormalLoadDistribution(params, ax, ay);
% DeltaFz_long = 60 * 2.0 * 0.30 / 0.80 = 45.0 N
% Front loses 22.5 N per wheel (147.15 - 22.5 = 124.65 N)
% Rear gains 22.5 N per wheel (147.15 + 22.5 = 169.65 N)
expected_Fz2 = [124.65; 124.65; 169.65; 169.65];

if max(abs(Fz2 - expected_Fz2)) < tol && Fz2(1) < 147.15 && Fz2(3) > 147.15
    fprintf('Test 2: Positive longitudinal acceleration... PASSED (front loses, rear gains load)\n');
else
    fprintf('Test 2: FAILED\n');
    all_tests_passed = false;
end

%% Test 3: Negative longitudinal acceleration (ax < 0, ay = 0)
ax = -2.0; ay = 0.0;
[Fz3, diag3] = dynamicNormalLoadDistribution(params, ax, ay);
expected_Fz3 = [169.65; 169.65; 124.65; 124.65];

if max(abs(Fz3 - expected_Fz3)) < tol && Fz3(1) > 147.15 && Fz3(3) < 147.15
    fprintf('Test 3: Negative longitudinal acceleration... PASSED (front gains, rear loses load)\n');
else
    fprintf('Test 3: FAILED\n');
    all_tests_passed = false;
end

%% Test 4: Positive lateral acceleration (ax = 0, ay > 0)
ax = 0.0; ay = 2.0;
[Fz4, diag4] = dynamicNormalLoadDistribution(params, ax, ay);
% DeltaFz_lat_front = 0.5 * 60 * 2.0 * 0.30 / 0.60 = 30.0 N
% DeltaFz_lat_rear  = 0.5 * 60 * 2.0 * 0.30 / 0.60 = 30.0 N
% Left wheels lose 30.0 N (147.15 - 30.0 = 117.15 N)
% Right wheels gain 30.0 N (147.15 + 30.0 = 177.15 N)
expected_Fz4 = [117.15; 177.15; 117.15; 177.15];

if max(abs(Fz4 - expected_Fz4)) < tol && Fz4(1) < 147.15 && Fz4(2) > 147.15
    fprintf('Test 4: Positive lateral acceleration... PASSED (right gains, left loses load)\n');
else
    fprintf('Test 4: FAILED\n');
    all_tests_passed = false;
end

%% Test 5: Negative lateral acceleration (ax = 0, ay < 0)
ax = 0.0; ay = -2.0;
[Fz5, diag5] = dynamicNormalLoadDistribution(params, ax, ay);
expected_Fz5 = [177.15; 117.15; 177.15; 117.15];

if max(abs(Fz5 - expected_Fz5)) < tol && Fz5(1) > 147.15 && Fz5(2) < 147.15
    fprintf('Test 5: Negative lateral acceleration... PASSED (left gains, right loses load)\n');
else
    fprintf('Test 5: FAILED\n');
    all_tests_passed = false;
end

%% Test 6: Combined positive ax and ay (ax > 0, ay > 0)
ax = 1.0; ay = 1.0;
[Fz6, diag6] = dynamicNormalLoadDistribution(params, ax, ay);
% dFz_long = 11.25 N (front loses, rear gains)
% dFz_lat  = 15.00 N (left loses, right gains)
% FL = 147.15 - 11.25 - 15.00 = 120.90 N (lowest load)
% FR = 147.15 - 11.25 + 15.00 = 150.90 N
% RL = 147.15 + 11.25 - 15.00 = 143.40 N
% RR = 147.15 + 11.25 + 15.00 = 173.40 N (highest load)
expected_Fz6 = [120.90; 150.90; 143.40; 173.40];

if max(abs(Fz6 - expected_Fz6)) < tol && Fz6(1) < Fz6(3) && Fz6(2) < Fz6(4)
    fprintf('Test 6: Combined positive ax and ay... PASSED (RR highest, FL lowest)\n');
else
    fprintf('Test 6: FAILED\n');
    all_tests_passed = false;
end

%% Test 7: Combined negative ax and ay (ax < 0, ay < 0)
ax = -1.0; ay = -1.0;
[Fz7, diag7] = dynamicNormalLoadDistribution(params, ax, ay);
% FL = 147.15 + 11.25 + 15.00 = 173.40 N (highest load)
% FR = 147.15 + 11.25 - 15.00 = 143.40 N
% RL = 147.15 - 11.25 + 15.00 = 150.90 N
% RR = 147.15 - 11.25 - 15.00 = 120.90 N (lowest load)
expected_Fz7 = [173.40; 143.40; 150.90; 120.90];

if max(abs(Fz7 - expected_Fz7)) < tol && Fz7(1) > Fz7(3) && Fz7(2) > Fz7(4)
    fprintf('Test 7: Combined negative ax and ay... PASSED (FL highest, RR lowest)\n');
else
    fprintf('Test 7: FAILED\n');
    all_tests_passed = false;
end

%% Test 8: Vertical-load conservation (sum(Fz) == m*g)
mg_total = params.m * params.g;
conserved = true;
test_accels = [
    0, 0;
    2.0, 0;
   -2.0, 0;
    0, 2.0;
    0, -2.0;
    1.5, 1.5;
   -1.5, 1.5;
    1.5, -1.5;
   -1.5, -1.5;
    3.0, 2.0;
];

for i = 1:size(test_accels, 1)
    Fz_c = dynamicNormalLoadDistribution(params, test_accels(i,1), test_accels(i,2));
    if abs(sum(Fz_c) - mg_total) > tol
        conserved = false;
    end
end

if conserved
    fprintf('Test 8: Vertical-load conservation... PASSED (sum(Fz) == %.2f N for all 10 cases)\n', mg_total);
else
    fprintf('Test 8: FAILED\n');
    all_tests_passed = false;
end

%% Test 9: Front/rear static moment consistency (asymmetric CG)
asym_params = params;
asym_params.lf = 0.30;
asym_params.lr = 0.50;
asym_params.L  = 0.80;
Fz_asym = dynamicNormalLoadDistribution(asym_params, 0, 0);
% Front axle static load = mg * (0.50 / 0.80) = 588.60 * 0.625 = 367.875 N (183.9375 N per wheel)
% Rear axle static load  = mg * (0.30 / 0.80) = 588.60 * 0.375 = 220.725 N (110.3625 N per wheel)
expected_asym = [183.9375; 183.9375; 110.3625; 110.3625];

if max(abs(Fz_asym - expected_asym)) < tol
    fprintf('Test 9: Asymmetric CG static consistency... PASSED (front=367.88 N, rear=220.73 N)\n');
else
    fprintf('Test 9: FAILED\n');
    all_tests_passed = false;
end

%% Test 10: Track-width dependence
wide_params = params;
wide_params.tf = 0.80; % Wider front track
wide_params.tr = 0.40; % Narrower rear track
[~, diag10] = dynamicNormalLoadDistribution(wide_params, 0, 2.0);

% dFz_lat_front with tf=0.80: 0.5 * 60 * 2.0 * 0.30 / 0.80 = 22.50 N
% dFz_lat_rear  with tr=0.40: 0.5 * 60 * 2.0 * 0.30 / 0.40 = 45.00 N
if abs(diag10.DeltaFz_lat_front - 22.50) < tol && abs(diag10.DeltaFz_lat_rear - 45.00) < tol
    fprintf('Test 10: Track-width dependence... PASSED (wider track reduces lateral transfer)\n');
else
    fprintf('Test 10: FAILED\n');
    all_tests_passed = false;
end

%% Test 11: CG-height dependence
low_cg_params = params;
low_cg_params.hCG = 0.15; % Half baseline CG height
high_cg_params = params;
high_cg_params.hCG = 0.45; % 1.5x baseline CG height

[~, diag_low] = dynamicNormalLoadDistribution(low_cg_params, 2.0, 2.0);
[~, diag_high] = dynamicNormalLoadDistribution(high_cg_params, 2.0, 2.0);

if abs(diag_low.DeltaFz_long - 22.50) < tol && abs(diag_high.DeltaFz_long - 67.50) < tol
    fprintf('Test 11: CG-height dependence... PASSED (load transfer scales linearly with hCG)\n');
else
    fprintf('Test 11: FAILED\n');
    all_tests_passed = false;
end

%% Test 12: Invalid negative-load condition (wheel lift rejection)
try
    % ay = 12.0 m/s^2 will produce lateral transfer = 180 N > 147.15 N, causing negative Fz_FL
    dynamicNormalLoadDistribution(params, 0, 12.0);
    fprintf('Test 12: FAILED (did not throw error for wheel lift)\n');
    all_tests_passed = false;
catch ME
    if strcmp(ME.identifier, 'dynamicNormalLoadDistribution:negativeNormalLoad')
        fprintf('Test 12: Negative-load rejection (wheel lift)... PASSED (error correctly caught)\n');
    else
        fprintf('Test 12: FAILED (wrong error identifier: %s)\n', ME.identifier);
        all_tests_passed = false;
    end
end

%% Test 13: Parameter and input validation
invalid_detected = true;
try
    dynamicNormalLoadDistribution(params, 'invalid', 0);
    invalid_detected = false;
catch
end
try
    bad_params = params;
    bad_params.m = -10;
    dynamicNormalLoadDistribution(bad_params, 0, 0);
    invalid_detected = false;
catch
end

if invalid_detected
    fprintf('Test 13: Input and parameter validation... PASSED (invalid inputs rejected)\n');
else
    fprintf('Test 13: FAILED\n');
    all_tests_passed = false;
end

%% Test 14: 9-argument direct call syntax vs struct call syntax
[Fz_struct, ~] = dynamicNormalLoadDistribution(params, 1.2, 0.8);
[Fz_direct, ~] = dynamicNormalLoadDistribution(params.m, params.g, params.lf, params.lr, ...
                                               params.tf, params.tr, params.hCG, 1.2, 0.8);

if max(abs(Fz_struct - Fz_direct)) < tol
    fprintf('Test 14: Struct syntax vs 9-argument syntax consistency... PASSED\n');
else
    fprintf('Test 14: FAILED\n');
    all_tests_passed = false;
end

fprintf('\n');
if all_tests_passed
    fprintf('============================================================\n');
    fprintf('  All 14 dynamic normal load distribution tests PASSED!\n');
    fprintf('============================================================\n');
else
    fprintf('============================================================\n');
    fprintf('  ONE OR MORE TESTS FAILED.\n');
    fprintf('============================================================\n');
end
