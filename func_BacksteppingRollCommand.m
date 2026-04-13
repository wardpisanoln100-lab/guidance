function [phi_c, nu, omega_e, phi_d] = func_BacksteppingRollCommand( ...
    phi, omega_d, omega_d_dot, echi, delta, V_chi, params)
% func_BacksteppingRollCommand - 反步法滚转指令计算 (论文 Eq.7-11)
%
% 将运动学控制律输出的期望航向角速度 omega_d 通过反步法设计
% 转为滚转角指令 phi_c，考虑滚转一阶动态特性。
%
% 输入 (单位):
%   phi        - 当前滚转角，单位: rad
%   omega_d    - 期望航向角速度，单位: rad/s
%   omega_d_dot - omega_d 的时间导数，单位: rad/s^2
%   echi       - 航向误差 (chi - chi_f)，单位: rad
%   delta      - 进场角，单位: rad
%   V_chi      - 水平速度，单位: m/s
%   params     - 参数结构体，字段:
%                  k_e            - 航向角速度误差增益 (Eq.11, k_e > 0)
%                  g              - 重力加速度，单位: m/s^2
%                  lambda_phi_hat - 滚转时间常数估计值，单位: s
%
% 输出 (单位):
%   phi_c      - 滚转角指令，单位: rad (限幅 ±pi/6)
%   nu         - 辅助控制量，单位: rad/s
%   omega_e    - 航向角速度跟踪误差，单位: rad/s
%   phi_d      - 期望滚转角，单位: rad

g = params.g;
k_e = params.k_e;
lambda_phi_hat = params.lambda_phi_hat;

% --- Eq.7: 期望滚转角 ---
phi_d = atan(V_chi * omega_d / g);

% --- 航向角速度误差 (Proposition 1 定义) ---
omega_e = (g / V_chi) * tan(phi) - omega_d;

% --- Eq.11: 辅助控制律 nu ---
sec_phi_sq = 1.0 / (cos(phi) ^ 2);
nu = (V_chi / (g * sec_phi_sq)) * (-k_e * omega_e - (echi - delta) + omega_d_dot);

% --- Eq.18: 滚转指令 (使用自适应估计值) ---
% phi_c = lambda_phi_hat * nu + phi
phi_c = lambda_phi_hat * nu + phi;

% --- 限幅: ±30° ---
phi_max = pi / 6;  % ±30° 典型固定翼滚转限制
phi_c = max(-phi_max, min(phi_max, phi_c));
end
