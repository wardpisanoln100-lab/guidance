"""
DRL 误差补偿 - TD3 训练脚本
用法:
    python drl_train_td3.py                  # 默认训练
    python drl_train_td3.py --episodes 2000  # 自定义幕数
"""

import argparse
import torch
import numpy as np
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from scipy.io import savemat
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ===================== 网络定义 =====================

class Actor(nn.Module):
    """Actor 网络: 8 维观测 -> 2 维动作"""

    def __init__(self, obs_dim=8, act_dim=2, hidden1=400, hidden2=300):
        super().__init__()
        self.fc1 = nn.Linear(obs_dim, hidden1)
        self.fc2 = nn.Linear(hidden1, hidden2)
        self.fc3 = nn.Linear(hidden2, act_dim)
        # scaling: tanh -> [50, pi/6]
        self.act_scale = torch.tensor([50.0, np.pi / 6])

    def forward(self, obs):
        h = F.relu(self.fc1(obs))
        h = F.relu(self.fc2(h))
        out = torch.tanh(self.fc3(h))
        return out * self.act_scale

    def get_action(self, obs, noise_std=0.0):
        with torch.no_grad():
            action = self.forward(obs)
            if noise_std > 0:
                action += noise_std * torch.randn_like(action)
                action = torch.clamp(action,
                    torch.tensor([-50.0, -np.pi / 6]),
                    torch.tensor([50.0, np.pi / 6]))
            return action


class Critic(nn.Module):
    """Critic 网络: (8维观测 + 2维动作) -> 1维 Q 值"""

    def __init__(self, obs_dim=8, act_dim=2, hidden1=400, hidden2=300):
        super().__init__()
        self.fc1 = nn.Linear(obs_dim + act_dim, hidden1)
        self.fc2 = nn.Linear(hidden1, hidden2)
        self.fc3 = nn.Linear(hidden2, 1)

    def forward(self, obs, act):
        x = torch.cat([obs, act], dim=-1)
        h = F.relu(self.fc1(x))
        h = F.relu(self.fc2(h))
        return self.fc3(h)


# ===================== Replay Buffer =====================

class ReplayBuffer:
    def __init__(self, capacity=int(1e6)):
        self.capacity = capacity
        self.ptr = 0
        self.size = 0
        self.obs = np.zeros((capacity, 8), dtype=np.float32)
        self.act = np.zeros((capacity, 2), dtype=np.float32)
        self.reward = np.zeros(capacity, dtype=np.float32)
        self.next_obs = np.zeros((capacity, 8), dtype=np.float32)
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


# ===================== TD3 Agent =====================

class TD3Agent:
    def __init__(self, args):
        self.actor = Actor()
        self.critic1 = Critic()
        self.critic2 = Critic()
        self.target_actor = Actor()
        self.target_critic1 = Critic()
        self.target_critic2 = Critic()

        # 复制权重
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
        self.action_scale = np.array([50.0, np.pi / 6])
        self.action_bias = np.array([0.0, 0.0])

    def select_action(self, obs, noise_std):
        obs_tensor = torch.FloatTensor(obs).unsqueeze(0)
        with torch.no_grad():
            action = self.actor.get_action(obs_tensor, noise_std)
        return action.squeeze(0).numpy()

    def train(self, buffer, batch_size):
        if len(buffer) < batch_size:
            return

        obs, act, reward, next_obs, done = buffer.sample(batch_size)

        # === 更新 Critic ===
        with torch.no_grad():
            next_action = self.target_actor(next_obs)
            noise = torch.randn_like(next_action) * self.policy_noise
            noise = torch.clamp(noise, -self.noise_clip, self.noise_clip)
            next_action = torch.clamp(
                next_action + noise,
                torch.tensor([-50.0, -np.pi / 6]),
                torch.tensor([50.0, np.pi / 6])
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

        # === 延迟更新 Actor ===
        self.total_steps += 1
        if self.total_steps % self.actor_delay == 0:
            actor_loss = -self.critic1(obs, self.actor(obs)).mean()
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            # Polyak 更新目标网络
            for param, target_param in zip(
                self.actor.parameters(), self.target_actor.parameters()
            ):
                target_param.data.copy_(
                    self.tau * param.data + (1 - self.tau) * target_param.data
                )
            for param, target_param in zip(
                self.critic1.parameters(), self.target_critic1.parameters()
            ):
                target_param.data.copy_(
                    self.tau * param.data + (1 - self.tau) * target_param.data
                )
            for param, target_param in zip(
                self.critic2.parameters(), self.target_critic2.parameters()
            ):
                target_param.data.copy_(
                    self.tau * param.data + (1 - self.tau) * target_param.data
                )

    def save(self, path):
        torch.save(self.actor.state_dict(), path)

    def export_mat(self, mat_path):
        """将 Actor 权重导出为 MATLAB .mat 文件"""
        state_dict = self.actor.state_dict()
        weights = {
            'W1': state_dict['fc1.weight'].numpy().T,  # MATLAB: (input, output)
            'b1': state_dict['fc1.bias'].numpy(),
            'W2': state_dict['fc2.weight'].numpy().T,
            'b2': state_dict['fc2.bias'].numpy(),
            'W3': state_dict['fc3.weight'].numpy().T,
            'b3': state_dict['fc3.bias'].numpy(),
            'act_scale': np.array([50.0, np.pi / 6]),
        }
        savemat(mat_path, weights)
        print(f"Actor 权重已导出为 {mat_path}")
        print(f"  W1: {weights['W1'].shape}")
        print(f"  W2: {weights['W2'].shape}")
        print(f"  W3: {weights['W3'].shape}")


# ===================== MATLAB 接口 =====================

def load_matlab_data(mat_path):
    """加载 MATLAB 导出的训练数据"""
    from scipy.io import loadmat
    data = loadmat(mat_path)
    print(f"加载数据: {mat_path}")
    print(f"  观测维度: {data['observations'].shape}")
    print(f"  动作维度: {data['actions'].shape}")
    print(f"  样本数量: {data['observations'].shape[1]}")
    return data


# ===================== 训练主循环 =====================

def train(args):
    print("=== TD3 训练启动 ===")

    agent = TD3Agent(args)
    buffer = ReplayBuffer(args.buffer_size)

    # 加载 MATLAB 数据（如果有）
    if args.data_file:
        data = load_matlab_data(args.data_file)
        n_samples = data['observations'].shape[1]
        for i in range(n_samples):
            buffer.push(
                data['observations'][:, i],
                data['actions'][:, i],
                float(data['rewards'][0, i]),
                data['next_obs'][:, i],
                float(data['dones'][0, i]),
            )
        print(f"已加载 {n_samples} 个样本到经验池")

    # 训练循环
    episode_rewards = []
    noise_std = args.noise_std
    n_episodes = args.episodes

    for ep in range(1, n_episodes + 1):
        # 随机初始状态
        obs = np.zeros(8, dtype=np.float32)
        obs[4] = np.random.uniform(-10, 10)  # ed
        obs[6] = np.random.uniform(-0.2, 0.2)  # echi
        obs[0] = 0.01 * np.sign(obs[4] + 1e-8)  # beta
        obs[1] = obs[0] * 80  # Vy

        ep_reward = 0
        done = False
        steps = 0
        max_steps = 2000

        while not done and steps < max_steps:
            action = agent.select_action(obs, noise_std)

            # 简化环境 step（Python 端模拟 MATLAB 动力学）
            next_obs = step_simulation(obs, action)
            reward = compute_reward_py(next_obs, action)

            buffer.push(obs, action, reward, next_obs, float(done))
            agent.train(buffer, args.batch_size)

            obs = next_obs
            ep_reward += reward
            steps += 1

            if abs(obs[4]) > 200:  # ed 过大
                done = True
                ep_reward -= 50

        episode_rewards.append(ep_reward)

        # 噪声衰减
        noise_std = max(0.01, noise_std - args.noise_decay)

        if ep % 50 == 0:
            avg = np.mean(episode_rewards[-50:])
            print(f"Episode {ep}/{n_episodes}, Avg Reward: {avg:.4f}, Noise: {noise_std:.4f}")

    # 保存结果
    agent.save('drl_actor.pth')
    agent.export_mat('drl_actor_weights.mat')

    # 绘制训练曲线
    plt.figure(figsize=(10, 6))
    plt.plot(np.convolve(episode_rewards, np.ones(50)/50, mode='valid'))
    plt.xlabel('Episode')
    plt.ylabel('Episode Reward (smoothed)')
    plt.title('TD3 Training Progress')
    plt.grid(True)
    plt.savefig('drl_training_curve.png', dpi=150)
    print("训练曲线已保存到 drl_training_curve.png")

    print("=== 训练完成 ===")


def step_simulation(obs, action):
    """Python 端简化环境 step"""
    # 从观测中提取状态
    beta = obs[0]
    Vy = obs[1]
    p = obs[2]
    r = obs[3]
    ed = obs[4]
    chi = obs[6]
    integral_ed = obs[7]

    # 简化动力学参数
    dt = 0.01
    V_inf = 80

    # 风场
    beta_w = 0.0  # 简化：无风场，让 agent 学习从初始扰动中恢复

    # 简化的状态转移
    beta_dot = -0.1 * beta + 0.01 * p - r + 0.123 * chi + 0.001 * beta_w
    p_dot = -0.5 * beta - 2.0 * p + 0.5 * r + 5.0 * action[0] * 0.01
    r_dot = 0.2 * beta + 0.1 * p - 0.8 * r + 2.0 * action[1] * 0.01

    beta_new = beta + beta_dot * dt
    p_new = p + p_dot * dt
    r_new = r + r_dot * dt
    chi_new = chi + p_new * dt

    Vy_new = beta_new * V_inf
    ed_new = ed + Vy_new * dt
    integral_ed_new = integral_ed + ed * dt

    next_obs = np.array([beta_new, Vy_new, p_new, r_new,
                         ed_new, 0.0, chi_new, integral_ed_new], dtype=np.float32)
    return next_obs


def compute_reward_py(obs, action):
    """Python 端奖励函数（与 MATLAB drl_full_reward 一致）"""
    ed_prime = obs[4]
    chi_prime = obs[6]
    Vy = obs[1]
    p = obs[2]
    r = obs[3]

    def piecewise_r(x):
        ax = abs(x)
        if ax <= 1:
            r1 = -x * x
            r2 = np.exp(-x * x)
        else:
            r1 = -x * x * np.exp(-x * x)
            r2 = np.exp(-x * x)
        return r1 + r2

    Ts = 0.01
    Tf = 20.0
    time_scale = Ts / Tf

    reward = (3.0 * piecewise_r(ed_prime)
            + 1.0 * piecewise_r(chi_prime)
            + 0.5 * piecewise_r(Vy)
            + 100 * (-p * p)
            + 100 * (-r * r)) * time_scale

    return reward


# ===================== 入口 =====================

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--episodes', type=int, default=1000)
    parser.add_argument('--actor_lr', type=float, default=1e-5)
    parser.add_argument('--critic1_lr', type=float, default=1e-4)
    parser.add_argument('--critic2_lr', type=float, default=5e-5)
    parser.add_argument('--gamma', type=float, default=0.99)
    parser.add_argument('--tau', type=float, default=0.005)
    parser.add_argument('--actor_delay', type=int, default=2)
    parser.add_argument('--noise_std', type=float, default=0.1)
    parser.add_argument('--noise_decay', type=float, default=0.05)
    parser.add_argument('--policy_noise', type=float, default=0.2)
    parser.add_argument('--noise_clip', type=float, default=0.5)
    parser.add_argument('--batch_size', type=int, default=256)
    parser.add_argument('--buffer_size', type=int, default=int(1e6))
    parser.add_argument('--data-file', type=str, default=None,
                        help='MATLAB 导出的训练数据路径')
    args = parser.parse_args()

    train(args)
