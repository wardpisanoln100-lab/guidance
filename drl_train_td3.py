"""
DRL 误差补偿 - TD3 训练脚本
用法:
    python drl_train_td3.py
    python drl_train_td3.py --episodes 2000
    目前版本仅支持训练，评估脚本另行编写中...（计划在 drl_eval_td3.py 中实现）
    当前动作惩罚改为了0.5，上一个版本是0.1
    奖励函数没有乘系数，所以跑出来的奖励值较大
"""

import argparse
import torch
import numpy as np
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from scipy.io import savemat
import matplotlib

try:
    import matplotlib.pyplot as plt
    plt.ion()  # 开启交互模式
except Exception:
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

interactive_mode = not matplotlib.is_interactive()

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"使用设备: {device}")


class Actor(nn.Module):
    """Actor 网络: 9 维观测 -> 2 维动作"""

    def __init__(self, obs_dim=9, act_dim=2, hidden1=400, hidden2=300):
        super().__init__()
        self.fc1 = nn.Linear(obs_dim, hidden1)
        self.fc2 = nn.Linear(hidden1, hidden2)
        self.fc3 = nn.Linear(hidden2, act_dim)
        # 动作缩放 (减小范围以匹配误差幅度)
        self.register_buffer("act_scale", torch.tensor([15.0, np.pi / 12]))
        self.register_buffer("act_min", torch.tensor([-15.0, -np.pi / 12]))
        self.register_buffer("act_max", torch.tensor([15.0, np.pi / 12]))
        # 观测归一化 (固定值，基于状态物理范围)
        self.register_buffer("obs_mean", torch.zeros(obs_dim))
        self.register_buffer("obs_std", torch.tensor([
            0.02,   # beta: 典型 ±0.04 rad，归一化后 ±2
            1.6,    # Vy: 典型 ±3 m/s，归一化后 ±2
            0.05,   # p: 典型 ±0.08 rad/s，归一化后 ±1.6
            0.05,   # r: 典型 ±0.08 rad/s，归一化后 ±1.6
            30.0,   # ed: 典型 ±60 m，归一化后 ±2
            10.0,   # es: 典型 ±15 m，归一化后 ±1.5
            0.26,   # echi: 典型 ±0.26 rad (~15°)，归一化后 ±1
            1.0,    # kappa: 占位 (kappa=0 时此维恒为 0)
            0.5,    # phi: 典型 ±0.5 rad，归一化后 ±1
        ]))

    def forward(self, obs):
        obs_norm = (obs - self.obs_mean) / self.obs_std
        h = F.relu(self.fc1(obs_norm))
        h = F.relu(self.fc2(h))
        out = torch.tanh(self.fc3(h))
        return out * self.act_scale

    def get_action(self, obs, noise_std=0.0):
        with torch.no_grad():
            action = self.forward(obs)
            if noise_std > 0:
                # 按动作尺度缩放噪声：noise_std 表示相对于满量程的比例
                noise = torch.randn_like(action) * noise_std * self.act_scale
                action = action + noise
                action = torch.clamp(action, self.act_min, self.act_max)
            return action


class Critic(nn.Module):
    """Critic 网络: (9维观测 + 2维动作) -> 1维 Q 值"""

    def __init__(self, obs_dim=9, act_dim=2, hidden1=400, hidden2=300):
        super().__init__()
        self.fc1 = nn.Linear(obs_dim + act_dim, hidden1)
        self.fc2 = nn.Linear(hidden1, hidden2)
        self.fc3 = nn.Linear(hidden2, 1)
        # 与 Actor 相同的观测归一化
        self.register_buffer("obs_mean", torch.zeros(obs_dim))
        self.register_buffer("obs_std", torch.tensor([
            0.02, 1.6, 0.05, 0.05, 30.0, 10.0, 0.26, 1.0, 0.5
        ]))
        self.register_buffer("act_scale", torch.tensor([15.0, np.pi / 12]))

    def forward(self, obs, act):
        obs_norm = (obs - self.obs_mean) / self.obs_std
        act_norm = act / self.act_scale
        x = torch.cat([obs_norm, act_norm], dim=-1)
        h = F.relu(self.fc1(x))
        h = F.relu(self.fc2(h))
        return self.fc3(h)


class ReplayBuffer:
    def __init__(self, capacity=int(1e6)):
        self.capacity = capacity
        self.ptr = 0
        self.size = 0
        self.obs = np.zeros((capacity, 9), dtype=np.float32)
        self.act = np.zeros((capacity, 2), dtype=np.float32)
        self.reward = np.zeros(capacity, dtype=np.float32)
        self.next_obs = np.zeros((capacity, 9), dtype=np.float32)
        self.done = np.zeros(capacity, dtype=np.float32)

    def push(self, obs, act, reward, next_obs, done):
        idx = self.ptr % self.capacity
        self.obs[idx] = obs
        self.act[idx] = act
        self.reward[idx] = reward
        self.next_obs[idx] = next_obs
        self.done[idx] = done
        self.ptr += 1
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size):
        idx = np.random.randint(self.size, size=batch_size)
        return (
            torch.FloatTensor(self.obs[idx]),
            torch.FloatTensor(self.act[idx]),
            torch.FloatTensor(self.reward[idx]).unsqueeze(1),
            torch.FloatTensor(self.next_obs[idx]),
            torch.FloatTensor(self.done[idx]).unsqueeze(1),
        )

    def __len__(self):
        return self.size


class TD3Agent:
    def __init__(self, args):
        self.actor = Actor().to(device)
        self.critic1 = Critic().to(device)
        self.critic2 = Critic().to(device)
        self.target_actor = Actor().to(device)
        self.target_critic1 = Critic().to(device)
        self.target_critic2 = Critic().to(device)

        self.target_actor.load_state_dict(self.actor.state_dict())
        self.target_critic1.load_state_dict(self.critic1.state_dict())
        self.target_critic2.load_state_dict(self.critic2.state_dict())

        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=args.actor_lr)
        self.critic1_optimizer = optim.Adam(self.critic1.parameters(), lr=args.critic1_lr)
        self.critic2_optimizer = optim.Adam(self.critic2.parameters(), lr=args.critic2_lr)

        self.gamma = args.gamma
        self.tau = args.tau
        self.actor_delay = args.actor_delay
        self.policy_noise = args.policy_noise
        self.noise_clip = args.noise_clip
        self.total_steps = 0
        self.action_scale = np.array([15.0, np.pi / 12])
        self.action_bias = np.array([0.0, 0.0])

    def select_action(self, obs, noise_std):
        obs_tensor = torch.FloatTensor(obs).unsqueeze(0).to(device)
        with torch.no_grad():
            action = self.actor.get_action(obs_tensor, noise_std)
        return action.squeeze(0).cpu().numpy()

    def train(self, buffer, batch_size):
        if len(buffer) < batch_size:
            return

        obs, act, reward, next_obs, done = buffer.sample(batch_size)
        obs, act, reward, next_obs, done = (
            obs.to(device),
            act.to(device),
            reward.to(device),
            next_obs.to(device),
            done.to(device),
        )

        with torch.no_grad():
            next_action = self.target_actor(next_obs)
            # 按比例缩放噪声：policy_noise 是相对于满量程的比例
            noise_scale = torch.tensor([15.0, np.pi / 12], device=device)
            noise = torch.randn_like(next_action) * self.policy_noise * noise_scale
            noise_clip_scale = torch.tensor([15.0, np.pi / 12], device=device) * self.noise_clip
            noise = torch.clamp(noise, -noise_clip_scale, noise_clip_scale)
            next_action = torch.clamp(
                next_action + noise,
                torch.tensor([-15.0, -np.pi / 12], device=device),
                torch.tensor([15.0, np.pi / 12], device=device),
            )
            target_q1 = self.target_critic1(next_obs, next_action)
            target_q2 = self.target_critic2(next_obs, next_action)
            target_q = torch.min(target_q1, target_q2)
            target_q = reward + (1 - done) * self.gamma * target_q

        current_q1 = self.critic1(obs, act)
        current_q2 = self.critic2(obs, act)

        critic1_loss = F.mse_loss(current_q1, target_q)
        critic2_loss = F.mse_loss(current_q2, target_q)

        self.critic1_optimizer.zero_grad()
        critic1_loss.backward()
        self.critic1_optimizer.step()

        self.critic2_optimizer.zero_grad()
        critic2_loss.backward()
        self.critic2_optimizer.step()

        self.total_steps += 1
        if self.total_steps % self.actor_delay == 0:
            actor_loss = -self.critic1(obs, self.actor(obs)).mean()
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            for param, target_param in zip(self.actor.parameters(), self.target_actor.parameters()):
                target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
            for param, target_param in zip(self.critic1.parameters(), self.target_critic1.parameters()):
                target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
            for param, target_param in zip(self.critic2.parameters(), self.target_critic2.parameters()):
                target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

    def save(self, path):
        torch.save(self.actor.state_dict(), path)

    def export_mat(self, mat_path):
        state_dict = {k: v.cpu() for k, v in self.actor.state_dict().items()}
        weights = {
            "W1": state_dict["fc1.weight"].numpy().T,
            "b1": state_dict["fc1.bias"].numpy(),
            "W2": state_dict["fc2.weight"].numpy().T,
            "b2": state_dict["fc2.bias"].numpy(),
            "W3": state_dict["fc3.weight"].numpy().T,
            "b3": state_dict["fc3.bias"].numpy(),
            "act_scale": np.array([15.0, np.pi / 12]),
            "obs_mean": state_dict["obs_mean"].numpy(),
            "obs_std": state_dict["obs_std"].numpy(),
        }
        savemat(mat_path, weights)
        print(f"Actor 权重已导出为 {mat_path}")
        print(f"  W1: {weights['W1'].shape}")
        print(f"  W2: {weights['W2'].shape}")
        print(f"  W3: {weights['W3'].shape}")


def load_matlab_data(mat_path):
    from scipy.io import loadmat

    data = loadmat(mat_path)
    print(f"加载数据: {mat_path}")
    print(f"  观测维度: {data['observations'].shape}")
    print(f"  动作维度: {data['actions'].shape}")
    print(f"  样本数量: {data['observations'].shape[1]}")
    return data


def train(args):
    print("=== TD3 训练启动 ===")

    agent = TD3Agent(args)
    buffer = ReplayBuffer(args.buffer_size)

    load_dynamics()

    if args.data_file:
        data = load_matlab_data(args.data_file)
        n_samples = data["observations"].shape[1]
        for i in range(n_samples):
            buffer.push(
                data["observations"][:, i],
                data["actions"][:, i],
                float(data["rewards"][0, i]),
                data["next_obs"][:, i],
                float(data["dones"][0, i]),
            )
        print(f"已加载 {n_samples} 个样本到经验池")

    episode_rewards = []
    episode_success = []  # 1: 完整跑完 3000 步未终止，0: 提前触发 |ed|>200
    eval_rewards = []
    eval_episodes = []
    rms_ed_list = []
    rms_echi_list = []

    # 4 子图布局：奖励、评估、RMS 误差、成功率
    plt.ion()
    fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
    fig.canvas.manager.set_window_title("TD3 训练进度")

    # 预创建所有线对象（避免 clear() 重绘）
    # 子图 0: 奖励
    raw_line, = axes[0].plot([], [], alpha=0.3, linewidth=0.8, color="gray", label="Raw")
    ma_line, = axes[0].plot([], [], linewidth=2, color="blue", label="MA(50)")
    axes[0].set_ylabel("Episode Reward")
    axes[0].legend(loc="lower right", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # 子图 1: 评估
    eval_line, = axes[1].plot([], [], "o-", linewidth=2, color="green")
    axes[1].set_ylabel("Eval Reward\n(no noise)")
    axes[1].grid(True, alpha=0.3)

    # 子图 2: RMS (twinx)
    ax2b = axes[2].twinx()
    rms_ed_line, = axes[2].plot([], [], linewidth=1.5, color="red", label="RMS(ed)")
    rms_echi_line, = ax2b.plot([], [], linewidth=1.5, color="orange", label="RMS(echi)")
    axes[2].set_ylabel("RMS(ed)", color="red", fontsize=9)
    axes[2].tick_params(axis="y", labelcolor="red")
    ax2b.set_ylabel("RMS(echi)", color="orange", fontsize=9)
    ax2b.tick_params(axis="y", labelcolor="orange")
    axes[2].legend([rms_ed_line, rms_echi_line], ["RMS(ed)", "RMS(echi)"], loc="upper right", fontsize=8)
    axes[2].grid(True, alpha=0.3)
    ax2b.grid(True, alpha=0.3)

    # 子图 3: 存活率
    survival_line, = axes[3].plot([], [], linewidth=1.5, color="purple")
    axes[3].set_ylabel("Survival Rate")
    axes[3].set_xlabel("Episode")
    axes[3].set_ylim(-0.05, 1.05)
    axes[3].grid(True, alpha=0.3)

    fig.suptitle("TD3 Training", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.canvas.draw()
    fig.canvas.flush_events()

    noise_std = args.noise_std
    n_episodes = args.episodes
    eval_interval = 50
    total_env_steps = 0

    # 固定评估初值：零初始误差，纯抗风扰（与论文一致）
    eval_configs = [
        {"ed": 0.0, "es": 0.0, "echi": 0.0, "kappa": 0.0},
    ]

    def run_eval(agent, configs):
        """无噪声评估，返回平均奖励"""
        rewards = []
        for cfg in configs:
            obs = np.zeros(9, dtype=np.float32)
            obs[7] = cfg["kappa"]
            s_internal = 0.0
            chi_f = 0.0
            chi_aircraft = 0.0
            ep_r = 0
            done = False
            wind_t = 0
            prev_a = None
            for _ in range(2000):
                if done:
                    break
                action = agent.select_action(obs, noise_std=0.0)
                next_obs, s_internal, chi_f, chi_aircraft, wind_t = \
                    step_simulation(obs, action, s_internal, cfg["kappa"], chi_f, chi_aircraft, wind_t)
                r = compute_reward_py(obs, action, next_obs, prev_a)
                if abs(next_obs[4]) > 200:
                    done = True
                obs = next_obs
                ep_r += r
                prev_a = action
            rewards.append(ep_r)
        return np.mean(rewards)

    for ep in range(1, n_episodes + 1):
        kappa = 0.0

        # 零初始误差 + 小随机扰动（模拟配平状态）
        obs = np.zeros(9, dtype=np.float32)
        obs[7] = kappa

        s_internal = 0.0
        chi_f = 0.0
        chi_aircraft = 0.0

        ep_reward = 0
        done = False
        steps = 0
        max_steps = 2000   # 20s 仿真（dt=0.01）
        prev_action = None
        wind_t = 0

        # 跟踪指标
        ed_history = []
        echi_history = []

        while not done and steps < max_steps:
            # Warm-up: 前 warmup_steps 步使用随机动作充分探索
            if total_env_steps < args.warmup_steps:
                action = np.random.uniform(
                    [-15.0, -np.pi / 12],
                    [15.0, np.pi / 12]
                ).astype(np.float32)
            else:
                action = agent.select_action(obs, noise_std)

            next_obs, s_internal, chi_f, chi_aircraft, wind_t = step_simulation(obs, action, s_internal, kappa, chi_f, chi_aircraft, wind_t)
            reward = compute_reward_py(obs, action, next_obs, prev_action)

            if abs(next_obs[4]) > 200:
                done = True

            buffer.push(obs, action, reward, next_obs, float(done))

            # Warm-up 结束后每 train_every 步训练一次
            if total_env_steps >= args.warmup_steps and total_env_steps % args.train_every == 0:
                agent.train(buffer, args.batch_size)

            obs = next_obs
            ep_reward += reward
            steps += 1
            total_env_steps += 1
            prev_action = action

            ed_history.append(abs(obs[4]))
            echi_history.append(abs(obs[6]))

        episode_rewards.append(ep_reward)
        rms_ed_list.append(np.sqrt(np.mean(np.array(ed_history) ** 2)))
        rms_echi_list.append(np.sqrt(np.mean(np.array(echi_history) ** 2)))
        # 成功定义：完整跑完 3000 步，未触发 |ed|>200 终止
        # 成功定义：完整跑完 2000 步（20s），未触发 |ed|>200 终止
        episode_success.append(0 if done else 1)

        noise_std = max(0.01, noise_std - args.noise_decay)

        # 无噪声评估
        if ep % eval_interval == 0:
            eval_r = run_eval(agent, eval_configs)
            eval_rewards.append(eval_r)
            eval_episodes.append(ep)

        if ep % 20 == 0:
            avg = np.mean(episode_rewards[-50:])
            sr = np.mean(episode_success[-50:])
            print(f"Episode {ep}/{n_episodes}, Avg Reward: {avg:.4f}, Noise: {noise_std:.4f}, "
                  f"RMS ed: {rms_ed_list[-1]:.2f}, RMS echi: {rms_echi_list[-1]:.4f}, Survival: {sr:.0%}")

            # 实时绘图更新（仅 set_data，不 clear）
            try:
                x_ep = list(range(1, len(episode_rewards) + 1))

                # 子图 0: 奖励
                raw_line.set_data(x_ep, episode_rewards)
                if len(episode_rewards) >= 50:
                    smoothed = np.convolve(episode_rewards, np.ones(50) / 50, mode="valid")
                    ma_line.set_data(list(range(50, len(episode_rewards) + 1)), smoothed)
                axes[0].relim()
                axes[0].autoscale_view()

                # 子图 1: 评估
                if len(eval_rewards) > 0:
                    eval_line.set_data(eval_episodes, eval_rewards)
                    axes[1].relim()
                    axes[1].autoscale_view()

                # 子图 2: RMS
                rms_ed_line.set_data(x_ep, rms_ed_list)
                rms_echi_line.set_data(x_ep, rms_echi_list)
                axes[2].relim()
                axes[2].autoscale_view()
                ax2b.relim()
                ax2b.autoscale_view()

                # 子图 3: 存活率
                window = 50
                if len(episode_success) >= window:
                    survival_hist = []
                    x_sr = []
                    for i in range(window, len(episode_success) + 1, 5):
                        chunk = episode_success[i-window:i]
                        survival_hist.append(np.mean(chunk))
                        x_sr.append(i)
                    survival_line.set_data(x_sr, survival_hist)
                    axes[3].relim()
                    axes[3].autoscale_view()
                    axes[3].set_ylim(-0.05, 1.05)

                fig.suptitle(f"TD3 Training — Episode {ep}/{n_episodes}", fontsize=13, fontweight="bold")
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            except Exception:
                pass

    agent.save("drl_actor.pth")
    agent.export_mat("drl_actor_weights.mat")

    # 最终保存静态图
    fig.suptitle("TD3 Training — Final", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig("drl_training_curve.png", dpi=150)
    print("训练曲线已保存到 drl_training_curve.png")

    print("=== 训练完成 ===")


_A = None
_B = None
_F = None
_V_INF = 80.0


def load_dynamics(mat_path="drl_dynamics_matrices.mat"):
    global _A, _B, _F
    import h5py

    with h5py.File(mat_path, "r") as f:
        _A = f["A"][()].T          # MATLAB 列优先 -> h5py 行优先读取，所有矩阵都需要转置
        _B = f["B"][()].T          # MATLAB (4,2) -> h5py (2,4) -> .T 恢复 (4,2)
        _F = f["F"][()].T.flatten()  # MATLAB (4,1) -> h5py (1,4) -> .T.flatten() -> (4,)
    print(f"动力学矩阵已加载: A={_A.shape}, B={_B.shape}, F={_F.shape}")


def step_simulation(obs, action, s_internal, kappa, chi_f, chi_aircraft, wind_t=None, beta_w_override=None):
    """
    Python 端环境 step，使用 Y-8 真实动力学 + 完整 AFCS 控制链

    obs: 9 维观测 [beta, Vy, p, r, ed, es, echi, kappa, phi]
    action: 2 维动作 [delta_ed, delta_echi]
    s_internal: 虚拟目标弧长（内部状态，不进入 obs）
    kappa: 路径曲率（场景参数，幕内固定）
    chi_f: 路径切向角（内部状态，不进入 obs）
    chi_aircraft: 飞机航向角（内部状态，不进入 obs）
    wind_t: 当前风场时间
    beta_w_override: 手动指定 beta_w（用于测试）

    Returns: next_obs, s_internal_next, chi_f_next, chi_aircraft_next, wind_t
    """
    global _A, _B, _F

    if _A is None:
        load_dynamics()

    dt = 0.01
    V_inf = _V_INF
    g = 9.81

    beta = obs[0]
    p = obs[2]
    r = obs[3]
    ed = obs[4]
    es = obs[5]
    echi = obs[6]
    kappa = obs[7]
    phi = obs[8]

    ed_prime = ed + action[0]
    echi_prime = echi + action[1]

    k = 0.002
    ks = 0.2
    k_omega = 0.12
    gamma_param = 800000
    chi_inf = np.pi / 2

    z = np.clip(2.0 * k * ed_prime, -60, 60)
    exp_z = np.exp(z)
    delta = -chi_inf * (exp_z - 1.0) / (exp_z + 1.0)

    delta_prime = -chi_inf * (4.0 * k * exp_z) / ((exp_z + 1.0) ** 2)

    s_dot = ks * es + V_inf * np.cos(echi_prime)

    d_val = echi_prime - delta
    if abs(d_val) > 1e-8:
        sinc_term = np.sin(d_val / 2.0) / (d_val / 2.0)
    else:
        sinc_term = 1.0 - (d_val ** 2) / 24.0

    omega_d = (
        -k_omega * (echi_prime - delta)
        + kappa * s_dot
        + delta_prime * (V_inf * np.sin(echi_prime) - kappa * es * s_dot)
        - (ed_prime * V_inf / gamma_param) * sinc_term * np.cos((echi_prime + delta) / 2.0)
    )

    phi_d = np.arctan(V_inf * omega_d / g)
    phi_g = np.clip(phi_d, -np.pi / 6, np.pi / 6)

    phi_sat = np.clip(phi, -np.pi / 6, np.pi / 6)

    delta_a = (phi_g - phi_sat) + p
    delta_r = -0.019 * phi_sat - 0.03 * r

    delta_a = np.clip(delta_a, -0.5236, 0.5236)
    delta_r = np.clip(delta_r, -0.5236, 0.5236)

    u = np.array([delta_a, delta_r])

    if beta_w_override is not None:
        beta_w = beta_w_override
        if wind_t is None:
            wind_t = 0
    else:
        if wind_t is None:
            wind_t = 0
        wind_t += dt
        # 阵风从 t=0 起持续 10s（与论文一致）
        T_gust = 10.0
        if 0.0 <= wind_t <= T_gust:
            # 16 m/s 侧风速度 -> 等效侧滑角 beta_w = v_w / V_inf
            v_wind = 16.0 * (1 - np.cos(2 * np.pi * wind_t / T_gust)) / 2
            beta_w = v_wind / V_inf
        else:
            beta_w = 0.0

    x = np.array([beta, p, r, phi])

    def x_dot(state, ctrl, w):
        return _A @ state + _B @ ctrl + _F * w

    k1 = x_dot(x, u, beta_w)
    k2 = x_dot(x + 0.5 * dt * k1, u, beta_w)
    k3 = x_dot(x + 0.5 * dt * k2, u, beta_w)
    k4 = x_dot(x + dt * k3, u, beta_w)
    x_next = x + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

    Vy_next = x_next[0] * V_inf + beta_w * V_inf * 0.5

    # S-F 误差动力学：用局部 Serret-Frenet 方程推进 es, ed, echi
    # s_dot 已在上方计算（第 377 行），复用即可

    # 2) 飞机航迹角速度
    chi_aircraft_dot = g * np.tan(x_next[3]) / V_inf

    # 3) S-F 误差导数（用真实误差状态，非补偿后误差）
    es_dot = V_inf * np.cos(echi) - (1.0 - kappa * ed) * s_dot
    ed_dot = V_inf * np.sin(echi) - kappa * es * s_dot
    echi_dot_val = chi_aircraft_dot - kappa * s_dot

    # 4) 积分推进
    es_next = es + es_dot * dt
    ed_next = ed + ed_dot * dt
    echi_next = echi + echi_dot_val * dt
    # echi 卷绕到 [-pi, pi]
    echi_next = ((echi_next + np.pi) % (2 * np.pi)) - np.pi

    # 虚拟目标弧长推进
    s_internal_next = s_internal + s_dot * dt

    # 路径切向角推进（圆弧路径：d(chi_f)/ds = kappa）
    chi_f_next = chi_f + kappa * s_dot * dt
    # chi_f 卷绕到 [-pi, pi]
    chi_f_next = ((chi_f_next + np.pi) % (2 * np.pi)) - np.pi

    # 飞机航向角推进
    chi_aircraft_next = chi_aircraft + chi_aircraft_dot * dt
    # chi_aircraft 卷绕到 [-pi, pi]
    chi_aircraft_next = ((chi_aircraft_next + np.pi) % (2 * np.pi)) - np.pi

    next_obs = np.array(
        [x_next[0], Vy_next, x_next[1], x_next[2], ed_next, es_next, echi_next, kappa, x_next[3]],
        dtype=np.float32,
    )

    return next_obs, s_internal_next, chi_f_next, chi_aircraft_next, wind_t


def compute_reward_py(prev_obs, action, next_obs, prev_action=None):
    # 先归一化到 ~[-1,1]，使 piecewise_r 的分段点 c2=0.5 对所有维度均合理
    next_ed = float(next_obs[4]) / 30.0       # 30 m
    next_echi = float(next_obs[6]) / 0.26     # 0.26 rad ≈ 15°
    next_Vy = float(next_obs[1]) / 1.6        # 1.6 m/s
    next_p = float(next_obs[2]) / 0.05        # 0.05 rad/s
    next_r = float(next_obs[3]) / 0.05        # 0.05 rad/s

    def piecewise_r(x):
        """分段奖励：|x|<0.5 二次+高斯奖励，|x|>0.5 线性惩罚"""
        c1 = -1.0
        c2 = 0.5
        c3 = 3.0
        c4 = -3.0

        if abs(x) < c2:
            r1 = c1 * x * x
        else:
            r1 = 2.0 * c1 * c2 * np.sign(x) * x - c1 * c2 * c2
        r2 = c3 * np.exp(c4 * x * x)
        return r1 + r2

    reward = (
        3.0 * piecewise_r(next_ed)
        + 1.0 * piecewise_r(next_echi)
        + 0.5 * piecewise_r(next_Vy)
        + 0.5 * piecewise_r(next_p)
        + 0.5 * piecewise_r(next_r)
    )

    if prev_action is not None:
        # 动作变化率惩罚：按动作量程归一化，系数需与主奖励(~16.5/步)可比
        delta_ed_norm = (action[0] - prev_action[0]) / 15.0
        delta_echi_norm = (action[1] - prev_action[1]) / (np.pi / 12)
        reward += -1.0 * delta_ed_norm * delta_ed_norm
        reward += -1.0 * delta_echi_norm * delta_echi_norm

    return reward


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=2000)
    parser.add_argument("--actor_lr", type=float, default=3e-4)
    parser.add_argument("--critic1_lr", type=float, default=3e-4)
    parser.add_argument("--critic2_lr", type=float, default=3e-4)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--tau", type=float, default=0.005)
    parser.add_argument("--actor_delay", type=int, default=2)
    parser.add_argument("--noise_std", type=float, default=0.1)
    parser.add_argument("--noise_decay", type=float, default=5e-5)
    parser.add_argument("--policy_noise", type=float, default=0.1)
    parser.add_argument("--noise_clip", type=float, default=0.3)
    parser.add_argument("--batch_size", type=int, default=256)
    parser.add_argument("--buffer_size", type=int, default=int(1e6))
    parser.add_argument("--warmup_steps", type=int, default=10000, help="随机探索预热步数")
    parser.add_argument("--train_every", type=int, default=4, help="每 N 步训练一次")
    parser.add_argument("--data-file", type=str, default=None, help="MATLAB 导出的训练数据路径")
    args = parser.parse_args()

    train(args)
