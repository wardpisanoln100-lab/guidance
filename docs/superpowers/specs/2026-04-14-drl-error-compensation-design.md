# DRL 误差补偿设计文档

> 编写日期: 2026-04-14
> 状态: 待审核

## 1. 背景与动机

### 1.1 参考论文

朱越, 王睿, 周洲. "基于深度强化学习的中小型无人机高原峡谷抗风飞行控制." 西北工业大学学报, 2026, 44(1): 1-11.

论文核心：以 L1 制导律为基准，用 TD3 深度强化学习输出补偿量 [y_c, χ_c] 修正 L1 的目标参考点。在最大侧风 16 m/s 下，轨迹偏差仅为传统 L1 的 28.6%。

### 1.2 当前问题

Y-8 项目在 RNP AR 进场仿真中，现有 Serret-Frenet + 运动学控制律 (Eq.5, 6a, 6b) 在风场扰动下的表现需要提升。论文证明了 DRL 补偿式方法的有效性，且补偿式（而非取代式）保留了原有控制律的稳定性保证。

### 1.3 设计目标

新建独立的 DRL 训练与推理模块，通过调用现有函数（`system_model_update`、`func_KinematicControlLaw`）的方式，在误差计算和运动学控制律之间插入 DRL 补偿，补偿 Serret-Frenet 误差信号（ed, echi）。

**约束: 不修改任何现有 .m 文件，全部通过新建文件 + 调用的方式实现。**

---

## 2. 整体架构

### 2.1 数据流

```
飞机状态 [lat, lon, chi, V, β, Vy, p, r, φ, ...]
      │
      ├── 调用 system_model_update() ──→ [es, ed, echi]
      │
      └── 从飞机模型读取 ───────────────→ [β, Vy, p, r]
                                               │
                                               ▼
                                    拼接观测: [β, Vy, p, r, ed, es, echi, ∫ed dt]
                                               │
                                               ▼
                                    ┌─────────────────────┐
                                    │ drl_error_compensation │
                                    │ (新建，不调用现有函数) │
                                    │ 输出: [Δed, Δechi]   │
                                    └──────────┬────────────┘
                                               │
                                               ▼
                                    补偿: ed' = ed + Δed
                                          echi' = echi + Δechi
                                               │
                                               ▼
                                    调用 func_KinematicControlLaw(es, ed', echi', V, kappa)
                                               │
                                               ▼
                                          omega_d
```

### 2.2 设计原则

1. **不修改现有文件**: 所有 DRL 功能在新文件中实现
2. **调用而非复制**: 通过调用 `system_model_update` 和 `func_KinematicControlLaw` 复用现有逻辑
3. **补偿而非取代**: 保留 Serret-Frenet 和运动学控制律的完整结构
4. **可开关**: 通过一个开关变量控制 DRL 是否生效，方便对比实验

---

## 3. DRL 策略设计

### 3.1 算法

TD3 (Twin Delayed Deep Deterministic Policy Gradient)

### 3.2 观测空间（8 维）

```
S = [β, V_y, p, r, ed, es, echi, ∫ed dt]
```

| 维度 | 符号 | 单位 | 来源 | 选择理由 |
|------|------|------|------|---------|
| 1 | β | rad | 飞机状态 | 侧滑角，风扰动最先影响的状态量 |
| 2 | V_y | m/s | 飞机状态 | 侧向速度，直接反映侧风影响 |
| 3 | p | rad/s | 飞机状态 | 滚转角速度，反映姿态扰动 |
| 4 | r | rad/s | 飞机状态 | 偏航角速度，反映姿态扰动 |
| 5 | ed | m | S-F 误差 | 横向误差，知道当前偏差大小 |
| 6 | es | m | S-F 误差 | 纵向误差，辅助信息（不参与补偿） |
| 7 | echi | rad | S-F 误差 | 航向误差，参与补偿 |
| 8 | ∫ed dt | m·s | 积分项 | 消除稳态偏差 |

### 3.3 动作空间（2 维）

```
A = [Δed, Δechi]

补偿后:
  ed'   = ed   + Δed      (横向误差补偿)
  echi' = echi + Δechi    (航向误差补偿)
```

动作范围限制（通过 Actor 输出层 tanh + scaling）：

| 动作 | 范围 | 依据 |
|------|------|------|
| Δed | ±50 m | RNP 容差 ≈ 556m，50m 补偿量足够 |
| Δechi | ±π/6 rad (±30°) | 航向补偿不宜过大 |

### 3.4 奖励函数

#### 分段函数 r(x)（论文公式 9，c 参数与论文一致）

```
c₁ = -1,  c₂ = 1,  c₃ = 1,  c₄ = -1

           { -x²,              |x| < 1
r₁(x)  =   {
           { -x²·exp(-x²),      |x| ≥ 1

r₂(x)  =  exp(-x²)

r(x)  =  r₁(x) + r₂(x)

特性:
  x = 0:     r = 1.0    ← 最大奖励
  x = 0.5:   r ≈ 0.529
  x = 1.0:   r ≈ -0.632 ← 转折点（连续）
  x = 2.0:   r ≈ -0.127
  x → ∞:     r → 0⁻     ← 趋近于 0（从负侧）
```

**重要**: r(x) 对大偏差的惩罚实际上趋于 0，不是趋于 -∞。这是论文的设计特性。大偏差时主要通过 p、r 惩罚项间接约束。

#### 完整评价函数（参考论文公式 11）

```
r(t_i) = [ 3.0  · r(ed')           +  ← 横向误差奖励（ed'=0 时 +3）
           1.0  · r(echi')         +  ← 航向误差奖励（echi'=0 时 +1）
           0.5  · r(V_y)           +  ← 侧向速度奖励（V_y=0 时 +0.5）
           100  · (-p²)            +  ← 滚转角速度惩罚（p≠0 时负值）
           100  · (-r²)            +  ← 偏航角速度惩罚（r≠0 时负值）
           0.01 · (-Δed_dot²)      +  ← Δed 变化率惩罚
           0.4  · (-Δechi_dot²)    ] · (T_s / T_f)
```

**系数正负号验证**:
- 奖励项: 正系数 × r(x)，当误差小时 r(x)→1，贡献为正 ✓
- 惩罚项: 正系数 × (-x²)，当状态偏离时贡献为负 ✓
- 与论文表 3 符号约定一致 ✓

**具体数值验证**:

理想工况: ed'=0.5, echi'=0.01, V_y=0.1, p=0.02, r=0.01
```
r(0.5) = 0.5288, r(0.01)≈1.0, r(0.1)≈0.98
奖励 = 3×0.5288 + 1×1.0 + 0.5×0.98 = 1.586 + 1.0 + 0.49 = 3.076
惩罚 = -100×0.0004 - 100×0.0001 = -0.04 - 0.01 = -0.05
总 ≈ 3.03 ✓ (正奖励，合理)
```

严重偏差: ed'=5m, echi'=0.5rad, V_y=3m/s, p=0.14, r=0.1
```
r(5)≈0, r(0.5)=0.5288, r(3)≈0
奖励 = 0 + 0.5288 + 0 = 0.5288
惩罚 = -100×0.0196 - 100×0.01 = -1.96 - 1.0 = -2.96
总 ≈ -2.43
```

注意: ed'=5m 时 r(ed')≈0，不产生直接惩罚。这是论文的设计 — 大偏差通过 p、r 惩罚间接约束。

### 3.5 网络结构

#### Actor 网络

```
输入: 8 维 (观测 S)
隐藏层 1: 400 节点, ReLU
隐藏层 2: 300 节点, ReLU
输出: 2 维 (Δed, Δechi), tanh + scaling

scaling:
  Δed:   tanh → ×50    (范围 ±50m)
  Δechi: tanh → ×π/6   (范围 ±30°)
```

#### Critic 网络 (×2, 结构相同, 独立更新)

```
输入: 8 维 (观测 S) + 2 维 (动作 A) = 10 维
隐藏层 1: 400 节点, ReLU
隐藏层 2: 300 节点, ReLU
输出: 1 维 (Q 值)
```

### 3.6 TD3 超参数

| 参数 | 值 | 来源 |
|------|---|------|
| Critic 1 学习率 | 1×10⁻⁴ | 论文表 4 |
| Critic 2 学习率 | 5×10⁻⁵ | 论文表 4 |
| Actor 学习率 | 1×10⁻⁵ | 论文表 4 |
| Actor 延迟更新步数 | 2 | 论文表 4 |
| Actor 噪声方差 | 0.1 | 论文表 4 |
| 噪声衰减率 | 0.05 | 论文表 4 |
| Q 值折扣因子 γ | 0.99 | 论文表 4 |
| 经验池大小 | 1×10⁶ | TD3 默认 |
| Batch size | 256 | TD3 默认 |

---

## 4. 训练策略

### 4.1 训练层次

| 层次 | 模型 | 用途 | 风场 |
|------|------|------|------|
| **Level 1** | 简化横航向线性模型 | 策略训练 | 1-cos 突风 |
| **Level 2** | AFCSModel 6DOF 全模型 | 策略迁移验证 | 高原峡谷风场 + Dryden 紊流 |
| **Level 3** | 半物理仿真平台 | 工程验证（可选） | 同 Level 2 |

### 4.2 Level 1 简化动力学模型

```
横航向小扰动方程（论文公式 12）：

x' = A·x + B·u + F·β_w
y  = C·x + G·β_w

状态: x = [β, p, r, φ]^T
输入: u = [δ_a, δ_r]^T
扰动: β_w = 风场引起的侧滑角

A、B 矩阵从 AirCraftData.m 的气动导数中提取
F = A(:,1) （论文公式 13）
```

### 4.3 训练回合设置

| 参数 | 值 | 说明 |
|------|---|------|
| 每幕最大步长 | 2000 | 约 20-40 秒仿真（dt=0.01-0.02） |
| 训练幕数 | 1000 | 论文中约 600 幕收敛 |
| 初始条件 | 随机 | ed ∈ [-10, 10]m, echi ∈ [-0.2, 0.2]rad |
| 风场 | 随机 | 最大侧风 5-16 m/s |

---

## 5. 代码集成

### 5.1 新增文件清单

| 文件 | 用途 | 依赖 |
|------|------|------|
| `drl_reward_function.m` | 奖励函数实现（r(x) 分段 + 完整评价函数） | 无 |
| `drl_env.m` | Level 1 训练环境封装 | 调用 `system_model_update`, `func_KinematicControlLaw` |
| `drl_train_td3.m` | TD3 训练脚本 | 调用 `drl_env.m`, `drl_reward_function.m` |
| `drl_error_compensation.m` | DRL 策略推理：输入 8 维观测 → 输出 2 维补偿 | 加载训练好的 .mat 权重 |
| `func_DRLCompensationBlock.m` | Simulink 封装（MATLAB Function 模块） | 调用 `drl_error_compensation.m` |
| `drl_run_comparison.m` | 对比实验脚本（DRL on/off） | 调用 AFCSModel |

### 5.2 与现有函数的调用关系

```
drl_env.m
  ├── 调用 system_model_update()    ← 获取 ed, es, echi
  ├── 调用 func_KinematicControlLaw() ← 计算 omega_d
  └── 调用 drl_reward_function()    ← 计算奖励

func_DRLCompensationBlock.m
  └── 调用 drl_error_compensation()  ← 策略推理

drl_error_compensation.m
  └── 加载 drl_actor_weights.mat     ← 网络权重（文件）

drl_run_comparison.m
  └── 调用 AFCSModel.mdl             ← 完整仿真
```

### 5.4 函数接口说明（供 Simulink 连线使用）

#### 5.4.1 `func_DRLCompensationBlock.m` — Simulink 封装入口

```matlab
function [delta_ed, delta_echi] = func_DRLCompensationBlock(beta, V_y, p, r, ed, es, echi, integral_ed, drl_enable)
```

| 端口 | 方向 | 维度 | 单位 | 说明 |
|------|------|------|------|------|
| beta | 输入 | 1 | rad | 侧滑角，从飞机模型读取 |
| V_y | 输入 | 1 | m/s | 侧向速度，从飞机模型读取 |
| p | 输入 | 1 | rad/s | 滚转角速度 |
| r | 输入 | 1 | rad/s | 偏航角速度 |
| ed | 输入 | 1 | m | S-F 横向误差，从 MonitorBlock 内部获取 |
| es | 输入 | 1 | m | S-F 纵向误差 |
| echi | 输入 | 1 | rad | 航向误差 |
| integral_ed | 输入 | 1 | m·s | ed 积分，需要在 Simulink 中用 Integrator 模块实现 |
| drl_enable | 输入 | 1 | bool | 开关，1=启用 DRL，0=关闭 |
| **delta_ed** | **输出** | 1 | m | DRL 输出的 ed 补偿量 |
| **delta_echi** | **输出** | 1 | rad | DRL 输出的 echi 补偿量 |

当 `drl_enable = 0` 时，输出 `[0, 0]`。

#### 5.4.2 Simulink 连线示意

```
飞机模型输出 ──┬── β ────────────────────────┐
               ├── V_y ──────────────────────┤
               ├── p ────────────────────────┤
               ├── r ────────────────────────┤
               │                             ▼
MonitorBlock ──├── ed ───────────────┐  func_DRLCompensationBlock
(内部已有的)   ├── es ───────────────┤  (新建 MATLAB Function 模块)
               ├── echi ─────────────┤
               │                     │
Integrator ────┤── ∫ed dt ───────────┤
(新建)         │                     │
Constant ──────┤── drl_enable ───────┤
               │                     │
               └─────────────────────┘
                       │
                       ├── delta_ed ──→ Sum 模块 (Add)
                       │                 输入1: ed (MonitorBlock 原始输出)
                       │                 输入2: delta_ed (DRL 输出)
                       │                 输出: ed' = ed + delta_ed
                       │
                       ├── delta_echi ──→ Sum 模块 (Add)
                       │                 输入1: echi (MonitorBlock 原始输出)
                       │                 输入2: delta_echi (DRL 输出)
                       │                 输出: echi' = echi + delta_echi
                       │
                       └── ed', echi' ──→ func_KinematicControlLaw
```

#### 5.4.3 需要新增的 Simulink 模块

| 模块 | 类型 | 连接 |
|------|------|------|
| `func_DRLCompensationBlock` | MATLAB Function | 见上表接口 |
| Integrator (ed 积分) | Continuous → Integrator | 输入: ed, 输出: ∫ed dt |
| Sum_ed | Math Operations → Sum | ed + delta_ed → ed' |
| Sum_echi | Math Operations → Sum | echi + delta_echi → echi' |
| Constant (drl_enable) | Sources → Constant | 值: 1 或 0 |
| 信号线: ed', echi' | 从 Sum 输出到 KinematicControlLaw | 替换原来的 ed, echi 输入 |

#### 5.4.4 调用链

```
func_DRLCompensationBlock
  └── 内部调用: drl_error_compensation(S, weights)
       └── 加载: drl_actor_weights.mat (网络权重)
            └── 纯矩阵运算: S * W1 → ReLU → * W2 → ReLU → * W3 → tanh → scaling
```

### 5.5 不修改的文件

以下现有文件**不做任何修改**：

- `system_model_init.m`
- `system_model_update.m`
- `func_SystemModelMonitorBlock.m`
- `func_KinematicControlLaw.m`
- `func_ApproachGuidance_RNPAR.m`

**注意**: `func_KinematicControlLaw` 的输入需要从原始的 `[es, ed, echi, ...]` 改为 `[es, ed', echi', ...]`。这通过 Simulink 连线实现（Sum 模块输出连接到 KinematicControlLaw），不需要修改 `.m` 文件本身。

---

## 6. 验证标准

### 6.1 Level 1 训练指标

| 指标 | 目标 | 依据 |
|------|------|------|
| 训练收敛幕数 | < 600 幕 | 论文结果 |
| 单幕平均奖励 | > 700 | 论文图 11 最终值约 700 |
| 最大 |ed'| | < 1m（简化模型下） | 论文线性模型结果 0.075m |

### 6.2 Level 2 迁移指标

| 指标 | 目标 | 对比基准 |
|------|------|---------|
| 最大 |ed| | < 无 DRL 时的 30% | 论文：28.6% |
| 最大 |echi| | < 无 DRL 时的 50% | — |
| 舵面高频振荡 | 无明显增加 | 论文：DRL 用舵量与 L1 相当 |

### 6.3 对比实验设计

| 实验 | DRL | 风场 | 目的 |
|------|-----|------|------|
| Exp-1 | 关闭 | 无风 | 基线（验证 DRL 不破坏无风性能） |
| Exp-2 | 关闭 | 高原峡谷风 | 当前基线表现 |
| Exp-3 | 开启 | 高原峡谷风 | DRL 抗风效果 |
| Exp-4 | 开启 | Dryden 紊流 | 鲁棒性验证 |

---

## 7. 风险与注意事项

### 7.1 r(x) 参数敏感度

第一版使用论文参数 c₂=1，ed 单位为米。Y-8 的 ed 可能远大于论文中 270kg 无人机的偏差。当 |ed|>2m 时，r(ed') 已接近 0，奖励项对大偏差的"直接惩罚"很弱，主要依赖 p、r 惩罚间接约束。

如果训练效果不佳，可能的调整方向：
1. 增大 c₂（如 30 或 50），让 r(x) 对中等偏差仍有显著奖励/惩罚
2. 增大 w₁（如 10），放大 ed 奖励项的量级
3. 增大 w₄, w₅（如 200），使 p、r 惩罚更敏感

### 7.2 简化动力学矩阵精度

Level 1 训练依赖从 `AirCraftData.m` 提取的 A、B、F 矩阵。矩阵不准确会导致训练策略迁移到 6DOF 时失效。

### 7.3 策略存储

训练好的网络权重以 `.mat` 格式存储：
- W1: 400×8 = 3,200 参数
- b1: 400×1 = 400 参数
- W2: 300×400 = 120,000 参数
- b2: 300×1 = 300 参数
- W3: 2×300 = 600 参数
- b3: 2×1 = 2 参数
- 总计: ~124,502 参数

---

## 8. 第一版范围

第一版只做以下范围，不包含：
- ~~es 补偿~~（暂不补偿纵向误差）
- ~~s_dot 补偿~~（暂不补偿推进速度）
- ~~完整风场建模~~（Level 1 用 1-cos 突风即可）
- ~~半物理仿真~~（可选后续步骤）

第一版交付（全部为新建文件）：
1. `drl_reward_function.m` — 奖励函数
2. `drl_env.m` — 训练环境
3. `drl_train_td3.m` — TD3 训练脚本
4. `drl_error_compensation.m` — 推理函数
5. `func_DRLCompensationBlock.m` — Simulink 封装
6. `drl_run_comparison.m` — 对比实验脚本
7. `drl_actor_weights.mat` — 训练好的策略网络权重
