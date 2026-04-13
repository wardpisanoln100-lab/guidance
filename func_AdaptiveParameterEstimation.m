function [lambda_hat, dlambda_dt] = func_AdaptiveParameterEstimation( ...
    lambda_hat, omega_e, echi, delta, omega_d_dot, params, dt)
% func_AdaptiveParameterEstimation - 自适应参数估计 (论文 Eq.22-24)
%
% 在线估计滚转闭环时间常数 lambda_phi，无需预先实验测定。
% 通过 Lyapunov 自适应律保证参数估计收敛到真实值。
%
% 输入 (单位):
%   lambda_hat  - 当前时间常数估计值，单位: s
%   omega_e     - 航向角速度跟踪误差，单位: rad/s
%   echi        - 航向误差 (chi - chi_f)，单位: rad
%   delta       - 进场角，单位: rad
%   omega_d_dot - 期望航向角速度导数，单位: rad/s^2
%   params      - 参数结构体，字段:
%                   k_a - 自适应增益 (Eq.24, k_a > 0)
%   dt          - 时间步长，单位: s
%
% 输出 (单位):
%   lambda_hat  - 更新后的时间常数估计值，单位: s
%   dlambda_dt  - 估计变化率，单位: s/s (无量纲)

k_a = params.k_a;

% --- Eq.24: 自适应更新律 ---
dlambda_dt = k_a * omega_e * ((echi - delta) - omega_d_dot);

% --- Euler 积分更新 ---
lambda_hat = lambda_hat + dlambda_dt * dt;

% --- 限幅: [0.05, 2.0] 秒 ---
lambda_hat_min = 0.05;   % 下限: 防止估计值过小导致数值不稳定
lambda_hat_max = 2.0;    % 上限: 防止估计值过大导致响应过慢
lambda_hat = max(lambda_hat_min, min(lambda_hat_max, lambda_hat));
end
