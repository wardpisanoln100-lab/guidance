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
