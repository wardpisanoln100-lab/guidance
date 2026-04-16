function [delta_ed, delta_echi] = drl_error_compensation(obs, weights_file)
% drl_error_compensation - DRL 策略推理：9 维观测 -> 2 维补偿
%
% 输入:
%   obs        : 9 维观测 [beta; Vy; p; r; ed; es; echi; kappa; phi]
%   weights_file: 权重 .mat 文件路径，默认 'drl_actor_weights.mat'
%
% 输出:
%   delta_ed   : 横向误差补偿量 (m), 范围 ±15
%   delta_echi : 航向误差补偿量 (rad), 范围 ±π/12

if nargin < 2 || isempty(weights_file)
    weights_file = 'drl_actor_weights.mat';
end

persistent W1 b1 W2 b2 W3 b3 obs_mean_val obs_std_val act_scale loaded_file
if isempty(W1) || ~strcmp(loaded_file, weights_file)
    if ~exist(weights_file, 'file')
        error('drl_error_compensation:NotFound', ...
            '权重文件 %s 不存在，请先运行 drl_train_td3.py', weights_file);
    end
    data = load(weights_file);
    W1 = data.W1;  % (9, 400)
    b1 = data.b1;  % (400,)
    W2 = data.W2;  % (400, 300)
    b2 = data.b2;  % (300,)
    W3 = data.W3;  % (300, 2)
    b3 = data.b3;  % (2,)
    if isfield(data, 'obs_mean')
        obs_mean_val = data.obs_mean(:)';
        obs_std_val = data.obs_std(:)';
    else
        obs_mean_val = zeros(1, 9);
        obs_std_val = ones(1, 9);
    end
    if isfield(data, 'act_scale')
        act_scale = data.act_scale(:)';
    else
        act_scale = [15, pi/12];
    end
    loaded_file = weights_file;
end

% 观测归一化
obs_norm = (obs' - obs_mean_val) ./ obs_std_val;

% 前向传播
h1 = max(0, obs_norm * W1 + b1);    % ReLU
h2 = max(0, h1 * W2 + b2);          % ReLU
out = tanh(h2 * W3 + b3);           % tanh

% Scaling
delta_ed = out(1) * act_scale(1);
delta_echi = out(2) * act_scale(2);
end
