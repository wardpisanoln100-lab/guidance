function drl_test_reward()
addpath(pwd);
fprintf('=== drl_reward_function 测试 ===\n');

r0 = drl_piecewise_reward(0);
assert(abs(r0 - 1.0) < 1e-10, sprintf('x=0 时 r 应为 1.0，实际 %.6f', r0));
fprintf('[PASS] r(0) = %.6f\n', r0);

r05 = drl_piecewise_reward(0.5);
assert(abs(r05 - 0.5288) < 0.001, sprintf('x=0.5 时 r 应约为 0.529，实际 %.6f', r05));
fprintf('[PASS] r(0.5) = %.6f\n', r05);

r1 = drl_piecewise_reward(1.0);
expected_r1 = -1 + exp(-1);
assert(abs(r1 - expected_r1) < 1e-6, sprintf('x=1 时 r 应为 %.6f，实际 %.6f', expected_r1, r1));
fprintf('[PASS] r(1.0) = %.6f\n', r1);

r5 = drl_piecewise_reward(5.0);
assert(abs(r5) < 0.001, sprintf('x=5 时 r 应接近 0，实际 %.6f', r5));
fprintf('[PASS] r(5.0) = %.6f (接近 0)\n', r5);

reward_ideal = drl_full_reward(0.5, 0.01, 0.1, 0.02, 0.01, 0.001, 0.0001);
assert(reward_ideal > 0, sprintf('理想工况奖励应 > 0，实际 %.6f', reward_ideal));
fprintf('[PASS] 理想工况奖励 = %.6f\n', reward_ideal);

reward_bad = drl_full_reward(5.0, 0.5, 3.0, 0.14, 0.1, 0.1, 0.05);
assert(reward_bad < 0, sprintf('严重偏差奖励应 < 0，实际 %.6f', reward_bad));
fprintf('[PASS] 严重偏差奖励 = %.6f\n', reward_bad);

fprintf('\n=== 所有测试通过 ===\n');
end
