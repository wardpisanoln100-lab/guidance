function [A, B, F, C, G] = drl_simplified_dynamics()
% drl_simplified_dynamics - 横航向小扰动线性模型（论文公式 12）
%
% x' = A*x + B*u + F*beta_w
% y  = C*x + G*beta_w
%
% 状态: x = [beta, p, r, phi]^T
% 输入: u = [delta_a, delta_r]^T
% 扰动: beta_w = 风场引起的侧滑角

% Y-8 飞行条件
V_inf = 80;          % 进场速度 (m/s)
mass = 54000;        % 质量 (kg)
S = 160;             % 参考面积 (m^2)
b = 42.4;            % 翼展 (m)
rho = 1.05;          % 高原密度 (kg/m^3)
I_x = 900000;        % 滚转惯量 (kg*m^2)
I_z = 2350000;       % 偏航惯量 (kg*m^2)
I_xz = 18000;        % 惯性积 (kg*m^2)

Gamma = I_x * I_z - I_xz^2;
Lambda1 = I_z / Gamma;
Lambda2 = I_x / Gamma;

q_bar = 0.5 * rho * V_inf^2;
b_2V = b / (2 * V_inf);

% 气动导数（无量纲，参考 Y-8 气动特性）
Y_beta = -0.405;
Y_p = -0.093;
Y_r = 0.290;
Y_delta_r = 0.229;

l_beta = -0.079;
l_p = -0.644;
l_r = 0.168;
l_delta_a = -0.290;
l_delta_r = 0.0017;

n_beta = 0.067;
n_p = -0.057;
n_r = -0.109;
n_delta_a = -0.075;
n_delta_r = -0.017;

% A 矩阵
A = zeros(4);
A(1,1) = (q_bar * S / (mass * V_inf)) * Y_beta;
A(1,2) = (q_bar * S / (mass * V_inf)) * Y_p * b_2V;
A(1,3) = (q_bar * S / (mass * V_inf)) * Y_r * b_2V - 1;
A(1,4) = 9.81 / V_inf;

A(2,1) = Lambda1 * q_bar * S * b * l_beta;
A(2,2) = Lambda1 * q_bar * S * b * l_p * b_2V;
A(2,3) = Lambda1 * q_bar * S * b * l_r * b_2V;

A(3,1) = Lambda2 * q_bar * S * b * n_beta;
A(3,2) = Lambda2 * q_bar * S * b * n_p * b_2V;
A(3,3) = Lambda2 * q_bar * S * b * n_r * b_2V;

A(4,2) = 1;

% B 矩阵
B = zeros(4, 2);
B(1,2) = (q_bar * S / (mass * V_inf)) * Y_delta_r;
B(2,1) = Lambda1 * q_bar * S * b * l_delta_a;
B(2,2) = Lambda1 * q_bar * S * b * l_delta_r;
B(3,1) = Lambda2 * q_bar * S * b * n_delta_a;
B(3,2) = Lambda2 * q_bar * S * b * n_delta_r;

% F 矩阵（论文公式 13: F = A(:,1)）
F = A(:, 1);

% C, G 矩阵（输出: ed 和 echi）
C = zeros(2, 4);
C(1, 1) = 1;  % beta 近似 ed 变化
C(2, 4) = 1;  % phi 近似 echi

G = zeros(2, 1);
end
