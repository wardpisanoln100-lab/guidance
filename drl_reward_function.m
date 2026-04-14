function [r, full] = drl_reward_function(x, ed_prime, echi_prime, V_y, p, r_val, delta_ed_dot, delta_echi_dot)
% drl_reward_function - DRL 奖励函数集合（论文公式 9 和 11）
%
% 用法:
%   r = drl_reward_function(x)
%     返回 drl_piecewise_reward(x) 的值
%
%   [r, full] = drl_reward_function(x, ed_p, echi_p, Vy, p, r_val, ded, dechi)
%     同时返回 piecewise r(x) 和完整评价函数值
%
% 直接调用子函数:
%   r = drl_piecewise_reward(x)
%   r = drl_full_reward(ed_p, echi_p, Vy, p, r_val, ded, dechi)

if nargout == 0 && nargin == 1
    r = drl_piecewise_reward(x);
    full = [];
elseif nargin >= 8
    r = drl_piecewise_reward(x);
    full = drl_full_reward(ed_prime, echi_prime, V_y, p, r_val, delta_ed_dot, delta_echi_dot);
else
    r = drl_piecewise_reward(x);
    full = [];
end
end

function r = drl_piecewise_reward(x)
% drl_piecewise_reward - 论文公式 9 的分段奖励函数 r(x)
%
% 参数: c1=-1, c2=1, c3=1, c4=-1
%
%           { -x^2,              |x| < 1
% r1(x) =   {
%           { -x^2 * exp(-x^2),   |x| >= 1
%
% r2(x) = exp(-x^2)
% r(x)  = r1(x) + r2(x)
%
% 特性:
%   x = 0:     r = 1.0    (最大奖励)
%   x = 0.5:   r ≈ 0.529
%   x = 1.0:   r ≈ -0.632 (转折点，连续)
%   x → ∞:     r → 0⁻     (趋近于 0，从负侧)

ax = abs(x);

if ax <= 1
    r1 = -x^2;
    r2 = exp(-x^2);
else
    r1 = -x^2 * exp(-x^2);
    r2 = exp(-x^2);
end

r = r1 + r2;
end

function reward = drl_full_reward(ed_prime, echi_prime, V_y, p, r_val, delta_ed_dot, delta_echi_dot)
% drl_full_reward - 论文公式 11 的完整评价函数
% 注意: 参数名用 r_val 避免与函数 r() 冲突
%
% r(t_i) = [ 3.0*r(ed') + 1.0*r(echi') + 0.5*r(V_y)
%          + 100*(-p^2) + 100*(-r^2)
%          + 0.01*(-delta_ed_dot^2) + 0.4*(-delta_echi_dot^2) ] * (Ts/Tf)

Ts = 0.01;
Tf = 20.0;
time_scale = Ts / Tf;

reward_ed = 3.0 * drl_piecewise_reward(ed_prime);
reward_echi = 1.0 * drl_piecewise_reward(echi_prime);
reward_Vy = 0.5 * drl_piecewise_reward(V_y);

penalty_p = 100 * (-p^2);
penalty_r = 100 * (-r_val^2);
penalty_delta_ed_dot = 0.01 * (-delta_ed_dot^2);
penalty_delta_echi_dot = 0.4 * (-delta_echi_dot^2);

raw_reward = reward_ed + reward_echi + reward_Vy ...
           + penalty_p + penalty_r ...
           + penalty_delta_ed_dot + penalty_delta_echi_dot;

reward = raw_reward * time_scale;
end
