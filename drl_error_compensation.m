function [delta_ed, delta_echi] = drl_error_compensation(obs, weights_file)
% drl_error_compensation - DRL 策略推理：8 维观测 -> 2 维补偿
%
% 输入:
%   obs        : 8 维观测 [beta; Vy; p; r; ed; echi; phi; integral_ed]
%   weights_file: 权重 .mat 文件路径，默认 'drl_actor_weights.mat'
%
% 输出:
%   delta_ed   : 横向误差补偿量 (m), 范围 ±50
%   delta_echi : 航向误差补偿量 (rad), 范围 ±π/6

if nargin < 2 || isempty(weights_file)
    weights_file = 'drl_actor_weights.mat';
end

persistent W1 b1 W2 b2 W3 b3 loaded_file
if isempty(W1) || ~strcmp(loaded_file, weights_file)
    if ~exist(weights_file, 'file')
        error('drl_error_compensation:NotFound', ...
            '权重文件 %s 不存在，请先运行 drl_train_td3.py', weights_file);
    end
    data = load(weights_file);
    W1 = data.W1;  % (8, 400)
    b1 = data.b1;  % (400,)
    W2 = data.W2;  % (400, 300)
    b2 = data.b2;  % (300,)
    W3 = data.W3;  % (300, 2)
    b3 = data.b3;  % (2,)
    loaded_file = weights_file;
end

% 前向传播
h1 = max(0, obs' * W1 + b1);       % ReLU
h2 = max(0, h1 * W2 + b2);         % ReLU
out = tanh(h2 * W3 + b3);          % tanh

% Scaling
act_scale = [50, pi/6];
delta_ed = out(1) * act_scale(1);
delta_echi = out(2) * act_scale(2);
end
