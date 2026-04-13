# 基于反步法的自适应路径制导 — 设计文档

> 论文来源: Kim, S.; Jung, D. "Adaptive Path Guidance Law for a Small Fixed-Wing UAS with Bounded Bank Angle." *Drones* 2025, 9, 180.
> 论文文件: `L1-BackStepping.pdf` / `L1-BackStepping_extracted.txt`

## 1. 现状分析

### 1.1 已实现部分

项目已完整实现论文中的 **运动学控制律** (Kinematic Control Law, Section 2.2.1):

| 论文方程 | 对应代码 | 功能 |
|----------|----------|------|
| Eq.5 | `func_KinematicControlLaw.m:28-37` | 进场角 `delta(ed)` |
| Eq.6a | `func_KinematicControlLaw.m:43-48` | 期望航向角速度 `omega_d` |
| Eq.6b | `func_KinematicControlLaw.m:40` | 虚拟目标推进速率 `s_dot` |

### 1.2 未实现部分

| 论文章节 | 方程 | 功能 | 优先级 |
|----------|------|------|--------|
| 2.2.2 Backstepping Design | Eq.7-11 | 将 `omega_d` 通过反步法转为滚转角指令 `phi_c`，考虑滚转一阶动态 | 高 |
| 2.2.3 Adaptive Parameter Estimation | Eq.17-24 | 在线估计滚转时间常数 `lambda_phi`，提高鲁棒性 | 高 |
| Lyapunov 稳定性证明 | Eq.12-15, 19-23 | 理论证明（不需要代码，但需理解以验证实现） | 参考 |

### 1.3 当前控制链路的缺口

```
当前:  system_model_update → KinematicControlLaw → omega_d → [直接送入Simulink，内部自行转换]
论文:  system_model_update → KinematicControlLaw → omega_d → Backstepping → phi_c → 执行机构
```

当前 `omega_d` 作为航向角速度直接输出给 Simulink，Simulink 内部的 AFCS 模型自行处理滚转。这种方式 **没有利用论文的反步法设计**，即没有考虑滚转一阶动态对航向跟踪效果的影响。

## 2. 架构设计

### 2.1 新增控制层

```
┌─────────────────────────────────────────────────────────────────┐
│                    func_SystemModelMonitorBlock                  │
│                                                                  │
│  system_model_update → errors (es, ed, echi)                    │
│       │                                                         │
│       ▼                                                         │
│  func_KinematicControlLaw → omega_d, s_dot, detail.delta        │
│       │                                                         │
│       ▼                                                         │
│  omega_d_dot = (omega_d - omega_d_prev) / dt   ← 有限差分       │
│       │                                                         │
│       ├───────────────────────────────┐                         │
│       ▼                               ▼                         │
│  func_AdaptiveParameterEstimation     func_BacksteppingRollCmd  │
│       │                               │                         │
│       │  lambda_phi_hat ──────────────┤                         │
│       │                               │                         │
│       │  omega_e, echi, delta,        │                         │
│       │  omega_d_dot ─────────────────┤                         │
│       │                               │                         │
│       ▼                               ▼                         │
│  [lambda_phi_hat 更新]            phi_c (滚转指令输出)           │
│                                                                  │
│  输出: [q_lat, q_lon, es, ed, echi, s, leg_idx, chi_f,          │
│         kappa, s_dot, omega_d, phi_c, lambda_phi_hat]           │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 文件变更清单

| 操作 | 文件 | 说明 |
|------|------|------|
| **新建** | `func_BacksteppingRollCommand.m` | 反步法滚转指令计算 (Eq.7-11) |
| **新建** | `func_AdaptiveParameterEstimation.m` | 自适应参数估计 (Eq.22-24) |
| **修改** | `system_model_init.m` | 新增 `k_e`, `k_a`, `lambda_phi_hat` 初始化 |
| **修改** | `func_SystemModelMonitorBlock.m` | 集成反步法层和自适应层 |
| **修改** | `plot_afcs_system_model_results.m` | 新增 `phi_c` 和 `lambda_phi_hat` 曲线 |
| **不动** | `func_KinematicControlLaw.m` | 已正确实现，无需修改 |

## 3. 函数详细设计

### 3.1 `func_BacksteppingRollCommand.m`

```matlab
function [phi_c, nu, omega_e, phi_d] = func_BacksteppingRollCommand( ...
    phi, omega_d, omega_d_dot, echi, delta, V_chi, params)
```

**输入:**
- `phi` — 当前滚转角 (rad)，从 Simulink/飞机状态获取
- `omega_d` — 期望航向角速度 (rad/s)，来自 KinematicControlLaw
- `omega_d_dot` — omega_d 的时间导数 (rad/s²)，用有限差分计算
- `echi` — 航向误差 (rad)，来自 system_model_update
- `delta` — 进场角 (rad)，来自 KinematicControlLaw 的 detail 输出
- `V_chi` — 水平速度 (m/s)
- `params` — 结构体，包含 `k_e`, `g`, `lambda_phi_hat`

**计算步骤:**

1. **Eq.7 — 期望滚转角:**
   ```matlab
   phi_d = atan(V_chi * omega_d / g);
   ```

2. **航向角速度误差 (Proposition 1 定义):**
   ```matlab
   omega_e = (g / V_chi) * tan(phi) - omega_d;
   ```

3. **Eq.11 — 辅助控制律 nu:**
   ```matlab
   nu = (V_chi / (g * sec(phi)^2)) * (-k_e * omega_e - (echi - delta) + omega_d_dot);
   ```

4. **Eq.18 — 滚转指令（使用自适应估计值）:**
   ```matlab
   phi_c = lambda_phi_hat * nu + phi;
   ```

5. **限幅:** 对 `phi_c` 限制在 ±30° (±pi/6 rad)，对应固定翼典型滚转限制

**输出:**
- `phi_c` — 滚转角指令 (rad)
- `nu` — 辅助控制量 (rad/s)
- `omega_e` — 航向角速度跟踪误差 (rad/s)
- `phi_d` — 期望滚转角 (rad)，用于调试/绘图

**数值安全:**
- `sec(phi)` 在 `|phi| → pi/2` 时发散，需确保 `|phi| < pi/2`（实际飞行自然满足）
- `V_chi = 0` 时跳过计算，返回零值

### 3.2 `func_AdaptiveParameterEstimation.m`

```matlab
function [lambda_hat, dlambda_dt] = func_AdaptiveParameterEstimation( ...
    lambda_hat, omega_e, echi, delta, omega_d_dot, params, dt)
```

**输入:**
- `lambda_hat` — 当前时间常数估计值 (s)
- `omega_e` — 航向角速度跟踪误差 (rad/s)
- `echi` — 航向误差 (rad)
- `delta` — 进场角 (rad)
- `omega_d_dot` — 期望航向角速度导数 (rad/s²)
- `params` — 结构体，包含 `k_a`
- `dt` — 时间步长 (s)

**计算步骤:**

1. **Eq.24 — 自适应更新律:**
   ```matlab
   dlambda_dt = k_a * omega_e * ((echi - delta) - omega_d_dot);
   ```

2. **Euler 积分更新:**
   ```matlab
   lambda_hat = lambda_hat + dlambda_dt * dt;
   ```

3. **限幅:** `lambda_hat = max(0.05, min(2.0, lambda_hat))`
   - 下限 0.05s: 防止估计值过小导致数值不稳定
   - 上限 2.0s: 防止估计值过大导致响应过慢

**输出:**
- `lambda_hat` — 更新后的时间常数估计值 (s)
- `dlambda_dt` — 估计变化率 (s/s)，用于调试

### 3.3 `system_model_init.m` 新增初始化

```matlab
% 反步法与自适应参数 (论文 Eq.11, Eq.24)
sys_model.k_e = 0.5;            % 航向角速度误差增益 (Eq.11 中的 k_e > 0)
sys_model.k_a = 0.01;           % 自适应增益 (Eq.24 中的 k_a > 0)
sys_model.lambda_phi_hat = 0.3; % lambda_phi 初始估计值 (秒)
```

### 3.4 `func_SystemModelMonitorBlock.m` 集成逻辑

核心改动：

```matlab
persistent sys_model lambda_phi_hat omega_d_prev

% ... 原有的 system_model_update 和 KinematicControlLaw 调用不变 ...

% 获取 detail.delta
[omega_d, ~, detail] = func_KinematicControlLaw(...);

% 计算 omega_d_dot (有限差分)
if isempty(omega_d_prev)
    omega_d_dot = 0;
else
    omega_d_dot = (omega_d - omega_d_prev) / max(dt, 1e-6);
    % 限幅防止数值噪声
    omega_d_dot = max(-10, min(10, omega_d_dot));
end
omega_d_prev = omega_d;

% 当前滚转角 phi (由 Simulink 模型提供)
phi = ...;  % 单位: rad，从 Simulink 飞机状态信号获取

% 自适应更新
[lambda_phi_hat, ~] = func_AdaptiveParameterEstimation( ...
    lambda_phi_hat, omega_e, errors.echi, detail.delta, omega_d_dot, ...
    struct('k_a', sys_model.k_a), dt);

% 反步法计算滚转指令
params_back = struct('k_e', sys_model.k_e, 'g', sys_model.g, ...
                     'lambda_phi_hat', lambda_phi_hat);
[phi_c, nu, omega_e, phi_d] = func_BacksteppingRollCommand( ...
    phi, omega_d, omega_d_dot, errors.echi, detail.delta, ...
    aircraft_V, params_back);

% 扩展输出: 第12路 phi_c, 第13路 lambda_phi_hat
y(12) = phi_c;
y(13) = lambda_phi_hat;
```

### 3.5 `plot_afcs_system_model_results.m` 新增绘图

```matlab
% 新增子图 9: 滚转指令 phi_c
subplot(3,4,9);
plot(time_vec, phi_c * rad2deg, 'm-', 'LineWidth', 1.5);
title(sprintf('滚转指令 phi_c (max=%.2f°)', max(abs(phi_c)*rad2deg)));

% 新增子图 10: 自适应参数 lambda_phi_hat
subplot(3,4,10);
plot(time_vec, lambda_phi_hat, 'c-', 'LineWidth', 1.5);
title(sprintf('自适应时间常数估计 lambda (final=%.4fs)', lambda_phi_hat(end)));

% 新增子图 11: 航向角速度跟踪误差 omega_e
subplot(3,4,11);
plot(time_vec, omega_e, 'k-', 'LineWidth', 1.5);
title(sprintf('航向角速度跟踪误差 omega_e (max=%.4f rad/s)', max(abs(omega_e))));
```

## 4. 参数选取指南

| 参数 | 符号 | 初值 | 调参方向 | 影响 |
|------|------|------|----------|------|
| 航向角速度误差增益 | k_e | 0.5 | 增大 → omega_e 收敛更快，但可能振荡 | Eq.11 |
| 自适应增益 | k_a | 0.01 | 增大 → 估计更快，但对噪声敏感 | Eq.24 |
| 滚转时间常数初值 | lambda_hat_0 | 0.3 s | 根据实际飞机阶跃响应测定 | Eq.18 |

**lambda_phi 物理意义:** 滚转闭环的一阶时间常数。对于 Y-8 级别的大型固定翼飞机，典型值 0.2-0.5s。可以通过以下方法粗略测定：
1. 给阶跃滚转指令
2. 记录实际滚转角达到 63.2% 的时间

## 5. Lyapunov 稳定性摘要（验证用）

论文的理论证明保证：

- **V1 (Eq.12):** `V1 = (1/2γ)es² + (1/2)ed² + (1/2)(echi-δ)² + (1/2)omega_e²`
- **V_dot_1 (Eq.15):** 在 Eq.11 辅助控制下，`V_dot_1 ≤ 0`（半负定）
- **V2 (Eq.19):** 加入参数估计误差项 `V2 = V1 + (1/(2k_a*lambda_phi))*lambda_tilde²`
- **V_dot_2 (Eq.23):** 在 Eq.24 自适应律下，`V_dot_2 ≤ 0`
- 由 **Barbalat 引理**：`es → 0, ed → 0, echi → 0, omega_e → 0` 当 `t → ∞`

代码实现需要保证的条件：
1. `V_chi ≠ 0`（速度非零，飞行中自然满足）
2. `|phi| < pi/2`（滚转角不超过 90°，正常飞行满足）
3. `lambda_phi > 0`（时间常数为正，通过限幅保证）
4. `k_e > 0, k_a > 0`（增益为正，初始化时保证）

## 6. Simulink 集成说明

> 以下仅供参考，具体由使用者自行搭建。

核心信号流变化：

```
原来: MonitorBlock → omega_d → [AFCS 自行处理]
现在: MonitorBlock → phi_c (滚转指令, rad) → AFCS 滚转通道
```

关键输入信号（需在 Simulink 中引入）：
- `phi` — 当前滚转角，单位 **rad**
- `omega_d` — 期望航向角速度，单位 **rad/s**（由 KinematicControlLaw 输出）
- `echi` — 航向误差，单位 **rad**
- `delta` — 进场角，单位 **rad**（由 KinematicControlLaw 的 detail 输出）
- `V_chi` — 水平速度，单位 **m/s**

关键输出信号：
- `phi_c` — 滚转角指令，单位 **rad**（限幅 ±30° 即 ±pi/6 rad）
- `lambda_phi_hat` — 滚转时间常数估计值，单位 **s**
- `omega_e` — 航向角速度跟踪误差，单位 **rad/s**（用于调试）

## 7. 验证计划

1. **离线 MATLAB 仿真：** 用 `Start.m` + 纯 MATLAB 循环验证新控制律
2. **对比测试：** 同一初始条件，分别运行"仅运动学层"和"完整反步法"，比较 ed/echi 收敛速度和稳态误差
3. **Simulink 仿真：** 将新控制律集成到 `AFCSModel.mdl` 中进行 HIL 仿真
4. **参数敏感性：** 变化 `lambda_phi_hat` 初始值 ±50%，验证自适应律的收敛性
