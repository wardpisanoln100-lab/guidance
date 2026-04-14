function drl_test_compensation()
fprintf('=== drl_error_compensation 测试 ===\n');

% 创建测试权重
W1 = randn(8, 400) * 0.01;
b1 = zeros(1, 400);   % row vector (1, 400)
W2 = randn(400, 300) * 0.01;
b2 = zeros(1, 300);   % row vector (1, 300)
W3 = randn(300, 2) * 0.01;
b3 = zeros(1, 2);     % row vector (1, 2)
act_scale = [50, pi/6];

save('drl_actor_weights_test.mat', 'W1', 'b1', 'W2', 'b2', 'W3', 'b3', 'act_scale');

% 清除 persistent 缓存
clear drl_error_compensation;

% 测试 1: 零观测
obs = zeros(8, 1);
[de, dchi] = drl_error_compensation(obs, 'drl_actor_weights_test.mat');
assert(abs(de) <= 50.01, sprintf('delta_ed 应 <= 50，实际 %.4f', de));
assert(abs(dchi) <= pi/6 + 0.01, sprintf('delta_echi 应 <= pi/6，实际 %.4f', dchi));
fprintf('[PASS] 零观测输出在范围内: de=%.4f, dchi=%.4f\n', de, dchi);

% 测试 2: 非零观测
obs2 = [0.05; 1.0; 0.02; 0.01; 5.0; 0; 0.1; 10.0];
[de2, dchi2] = drl_error_compensation(obs2, 'drl_actor_weights_test.mat');
fprintf('[INFO] 非零观测输出: de=%.4f, dchi=%.4f\n', de2, dchi2);

% 测试 3: 批量推理速度
obs_batch = randn(100, 8)';
tic;
for i = 1:100
    drl_error_compensation(obs_batch(:, i), 'drl_actor_weights_test.mat');
end
elapsed = toc;
fprintf('[INFO] 100 次推理耗时: %.4f 秒 (%.2f ms/次)\n', elapsed, elapsed/100*1000);

% 清理
delete('drl_actor_weights_test.mat');
clear drl_error_compensation;

fprintf('[PASS] 所有测试通过\n');
end
