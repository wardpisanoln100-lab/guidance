function data = drl_rollout_env(n_episodes)
% drl_rollout_env - MATLAB 端数据收集/环境 rollout
%
% 用法:
%   data = drl_rollout_env(100);  % 收集 100 幕数据
%
% 输出 data 结构体:
%   data.observations: (8, N) 观测数组
%   data.actions:      (2, N) 动作数组
%   data.rewards:      (1, N) 奖励数组
%   data.next_obs:     (8, N) 下一观测
%   data.dones:        (1, N) 终止标志
%
% 保存到: drl_training_data.mat

if nargin < 1, n_episodes = 100; end

% 加载环境组件
[A, B, F, ~, ~] = drl_simplified_dynamics();

% 环境参数
dt = 0.01;
max_steps = 2000;
V_inf = 80;
action_scale_ed = 50.0;
action_scale_echi = pi/6;

% 数据存储
obs_buf = [];
act_buf = [];
rew_buf = [];
next_buf = [];
done_buf = [];

for ep = 1:n_episodes
    % 随机初始条件
    ed0 = -10 + rand() * 20;
    echi0 = -0.2 + rand() * 0.4;

    % 动力学状态 x = [beta; p; r; phi]
    x = [0.01 * sign(ed0 + eps); 0; 0; echi0];
    y_lateral = ed0;
    chi = echi0;
    integral_ed = 0;
    wind_t = 0;

    prev_ed = ed0;
    prev_echi = echi0;

    for step = 1:max_steps
        % 构建观测
        Vy = x(1) * V_inf;
        obs = [x(1); Vy; x(2); x(3); y_lateral; 0; chi; integral_ed];

        % 随机探索动作（带噪声）
        noise_std = max(0.01, 5.0 * (1 - ep/n_episodes));  % 逐渐减小
        action_ed = randn() * noise_std;
        action_echi = randn() * noise_std * 0.1;
        action_ed = max(-action_scale_ed, min(action_scale_ed, action_ed));
        action_echi = max(-action_scale_echi, min(action_scale_echi, action_echi));
        action = [action_ed; action_echi];

        % 应用补偿
        ed_prime = y_lateral + action_ed;
        echi_prime = chi + action_echi;

        % 简 PD 控制器将补偿后误差映射为舵偏
        delta_a = 0.005 * ed_prime + 0.1 * x(2);
        delta_r = 0.02 * echi_prime + 0.1 * x(3);
        delta_a = max(-0.5, min(0.5, delta_a));
        delta_r = max(-0.5, min(0.5, delta_r));
        u = [delta_a; delta_r];

        % 风场（1-cos 突风）
        wind_t = wind_t + dt;
        peak_time = 10.0;
        duration = 10.0;
        max_wind = 16.0;
        if wind_t >= peak_time && wind_t <= peak_time + duration
            beta_w = max_wind * (1 - cos(2*pi*(wind_t - peak_time)/duration)) / 2;
        else
            beta_w = 0;
        end

        % RK4 推进
        k1 = A*x + B*u + F*beta_w;
        k2 = A*(x + 0.5*dt*k1) + B*u + F*beta_w;
        k3 = A*(x + 0.5*dt*k2) + B*u + F*beta_w;
        k4 = A*(x + dt*k3) + B*u + F*beta_w;
        x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

        % 侧向位移推进
        Vy_next = x_next(1) * V_inf + beta_w * V_inf * 0.5;
        y_lateral_next = y_lateral + Vy_next * dt;
        chi_next = chi + x_next(2) * dt;
        integral_ed_next = integral_ed + y_lateral * dt;

        % 计算奖励
        ed_dot = (y_lateral_next - prev_ed) / dt;
        echi_dot = (chi_next - prev_echi) / dt;
        reward = drl_full_reward(ed_prime, echi_prime, Vy_next, ...
            x_next(2), x_next(3), ed_dot, echi_dot);

        % 存储
        obs_buf = [obs_buf, obs];
        act_buf = [act_buf, action];
        rew_buf = [rew_buf, reward];

        next_obs = [x_next(1); Vy_next; x_next(2); x_next(3); y_lateral_next; 0; chi_next; integral_ed_next];
        next_buf = [next_buf, next_obs];

        done = 0;
        if abs(y_lateral_next) > 200
            done = 1;
            reward = reward - 50;
            rew_buf(end) = reward;
        end
        done_buf = [done_buf, done];

        % 更新状态
        x = x_next;
        y_lateral = y_lateral_next;
        chi = chi_next;
        integral_ed = integral_ed_next;
        prev_ed = y_lateral;
        prev_echi = chi;

        if done, break; end
    end

    if mod(ep, 20) == 0
        fprintf('Rollout: Episode %d/%d, Total samples: %d\n', ...
            ep, n_episodes, size(obs_buf, 2));
    end
end

data.observations = obs_buf;
data.actions = act_buf;
data.rewards = rew_buf;
data.next_obs = next_buf;
data.dones = done_buf;

save('drl_training_data.mat', '-struct', 'data');
fprintf('数据已保存到 drl_training_data.mat\n');
fprintf('总样本数: %d\n', size(obs_buf, 2));
end
