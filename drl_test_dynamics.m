function drl_test_dynamics()
fprintf('=== drl_simplified_dynamics 测试 ===\n');

[A, B, F, C, G] = drl_simplified_dynamics();

assert(isequal(size(A), [4 4]), sprintf('A 应为 4x4，实际 %dx%d', size(A)));
fprintf('[PASS] A 维度 = 4x4\n');

assert(isequal(size(B), [4 2]), sprintf('B 应为 4x2，实际 %dx%d', size(B)));
fprintf('[PASS] B 维度 = 4x2\n');

assert(isequal(size(F), [4 1]), sprintf('F 应为 4x1，实际 %dx%d', size(F)));
fprintf('[PASS] F 维度 = 4x1\n');

assert(isequal(size(C), [2 4]), sprintf('C 应为 2x4，实际 %dx%d', size(C)));
fprintf('[PASS] C 维度 = 2x4\n');

assert(isequal(size(G), [2 1]), sprintf('G 应为 2x1，实际 %dx%d', size(G)));
fprintf('[PASS] G 维度 = 2x1\n');

eig_vals = eig(A);
assert(all(real(eig_vals) < 0), sprintf('系统不稳定！特征值: %s', num2str(eig_vals')));
fprintf('[PASS] 系统稳定（所有特征值实部 < 0）\n');

fprintf('\n=== 所有测试通过 ===\n');
end
