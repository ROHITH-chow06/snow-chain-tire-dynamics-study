function verifySnowChainBenchmark()
% VERIFYSNOWCHAINBENCHMARK Verifies the mathematics and execution of the Phase 3K snow chain benchmark.
%
%   Runs a series of tests to ensure the traction multiplier logic works and 
%   the comparative wrapper executes without error.

    fprintf('============================================================\n');
    fprintf('  Running Verification: Snow Chain Benchmark (3K)\n');
    fprintf('============================================================\n\n');
    
    test_count = 0;
    all_tests_passed = true;
    
    % Test 1: Condition factory executes and produces expected structure
    test_count = test_count + 1;
    try
        cond = snowChainCondition();
        if strcmp(cond.type, 'SnowChain') && isfield(cond, 'pure_params') && isfield(cond, 'combined_params')
            fprintf('Test %d: Condition factory executes and produces expected structure... PASSED\n', test_count);
        else
            error('Invalid structure');
        end
    catch ME
        fprintf('Test %d: FAILED (%s)\n', test_count, ME.message);
        all_tests_passed = false;
    end
    
    % Test 2: SnowChain parameterization correctly applies the mathematical scaling factor
    test_count = test_count + 1;
    cond_snow = roadSurfaceCondition('Snow');
    if cond.pure_params.Dx > cond_snow.pure_params.Dx && cond.pure_params.Dy == cond_snow.pure_params.Dy
        fprintf('Test %d: SnowChain benchmark correctly applies the selected longitudinal enhancement while preserving baseline lateral grip... PASSED\n', test_count);
    else
        fprintf('Test %d: FAILED\n', test_count);
        all_tests_passed = false;
    end
    
    % Test 3: Benchmark wrapper executes without error
    test_count = test_count + 1;
    
    % Note: roverParameters() is the established generic parameter factory 
    % throughout the physics stack, supplying the four-wheel vehicle parameters.
    params = roverParameters();
    sim_options.t_span = [0, 0.5]; % Very short for fast test
    sim_options.dt = 0.01;
    
    try
        benchmark = runSnowChainBenchmark(params, sim_options);
        if isfield(benchmark, 'StraightAcceleration') && isfield(benchmark.StraightAcceleration, 'SnowChain')
            fprintf('Test %d: Benchmark wrapper executes without error... PASSED\n', test_count);
        else
            error('Missing fields');
        end
    catch ME
        fprintf('Test %d: FAILED (%s)\n', test_count, ME.message);
        all_tests_passed = false;
    end
    
    % Test 4: Acceleration is greater with Snow Chains
    test_count = test_count + 1;
    accel_snow = benchmark.StraightAcceleration.Snow.handling.final_speed;
    accel_chain = benchmark.StraightAcceleration.SnowChain.handling.final_speed;
    
    if accel_chain > accel_snow
        fprintf('Test %d: Selected chain benchmark produces greater simulated final speed under the acceleration scenario... PASSED\n', test_count);
    else
        fprintf('Test %d: FAILED (Chain: %g, Snow: %g)\n', test_count, accel_chain, accel_snow);
        all_tests_passed = false;
    end
    
    fprintf('\n============================================================\n');
    if all_tests_passed
        fprintf('  All %d snow chain benchmark tests PASSED!\n', test_count);
    else
        fprintf('  SOME TESTS FAILED IN SNOW CHAIN BENCHMARK.\n');
    end
    fprintf('============================================================\n\n');
end
