# Y8飞机模型 - 论文System Model与Guidance实现设计

**日期**: 2026-04-08
**主题**: 复现论文 "Adaptive Path Guidance Law for a Small Fixed-Wing UAS with Bounded Bank Angle" 的System Model与外环Guidance

---

## 1. 概述

本文档描述如何使用现有的Y8飞机模型（AFCSModel.mdl）实现论文中的System Model（2.1节）和外环Guidance（2.2.1节）。

### 1.1 设计目标
- 保留现有的AFCSModel.mdl飞机动力学模型
- 仅实现论文的外环Guidance（kinematic control law）
- 使用AFCSModel的21个输出计算S-F误差状态
- 复用RNP AR飞行计划的航段几何信息

---

## 2. 输入接口

### 2.1 来自AFCSModel的输入（21个输出端口）

| 端口号 | 名称 | 单位 | 用途 |
|--------|------|------|------|
| 1 | V | m/s | 空速 → Vχ |
| 7 | phi | rad | 滚转角 → 论文中的φ |
| 9 | psi | rad | 偏航角 |
| 10 | Latitude | deg | 纬度 → 位置x |
| 11 | Longitude | deg | 经度 → 位置y |
| 15 | Chi | deg | 航迹角 → 论文中的χ |

**注意**: 需要将角度单位从度转换为弧度（Chi, psi）

---

## 3. 系统模型实现

### 3.1 简化假设（论文2.1节）
- 固定翼UAS定高、定速飞行
- 使用简化unicycle kinematics
- 低层autopilot负责空速和高度控制

### 3.2 运动学方程（论文Eq. 1-3）

```matlab
% 飞机运动学
x_dot = Vχ * cos(χ)
y_dot = Vχ * sin(χ)
χ_dot = ω

% 滚转-航向关系
ω = (g / Vχ) * tan(φ)

% 闭式滚转动力学（一阶系统）
φ_dot = (1/λφ) * (φc - φ)
```

### 3.3 关键常数
- g = 9.8 m/s² (重力加速度)
- λφ: 滚转闭环时间常数（论文通过参数自适应估计）

---

## 4. Serret-Frenet (S-F) 误差状态计算

### 4.1 误差定义（论文Eq. 4）

```matlab
% 误差状态
% es: along-track error (纵向误差)
% ed: cross-track error (横向误差)
% eχ: 航迹角误差 = χ - χf

% 误差动力学
es_dot = Vχ * cos(eχ) - (1 - κ(s)*ed) * s_dot
ed_dot = Vχ * sin(eχ) - κ(s) * es * s_dot
eχ_dot = ω - κ(s) * s_dot
```

### 4.2 路径几何参数（来自RNP AR飞行计划）

| 航段类型 | q(s)位置 | χf切向方向 | κ曲率 |
|----------|----------|------------|-------|
| TF (直线) | 直线方程 | 常数 | 0 |
| RF (圆弧) | 圆心 ± R | 变化 | 1/R |

**实现方式**: 复用 `func_ApproachGuidance_RNPAR.m` 中的航段解析逻辑

---

## 5. Kinematic Control Law 实现

### 5.1 Approach Angle（论文Eq. 5）

```matlab
function delta = compute_approach_angle(ed, k, chi_inf)
% δ(ed) = -χ∞ * (e^(2*k*ed) - 1)/(e^(2*k*ed) + 1)
% 这是一个单调递减的奇函数
delta = -chi_inf * (exp(2*k*ed) - 1) ./ (exp(2*k*ed) + 1);
```

**参数**:
- k: 正增益
- χ∞: approach angle极限, ∈ (0, π/2), 默认 π/2

### 5.2 航向率命令（论文Eq. 6a）

```matlab
function omega_d = compute_omega_d(es, ed, echi, chi_f, V_chi, ...
                                    s_dot, kappa, delta, delta_prime, ...
                                    ks, k_omega, gamma)
% ωd = -kω*(~χ - δ) + κ*ṡ + δ'*(Vχ*sin(eχ) - κ*es*ṡ) ...
%      - (ed*Vχ/γ) * (sin(eχ) - sin(δ)) / (eχ - δ)

omega_d = -k_omega * (echi - delta) + kappa * s_dot ...
          + delta_prime * (V_chi * sin(echi) - kappa * es * s_dot) ...
          - (ed * V_chi / gamma) * sinc_half(echi - delta) ...
            * cos((echi + delta) / 2);
```

### 5.3 弧长变化率（论文Eq. 6b）

```matlab
function s_dot = compute_s_dot(es, V_chi, echi, ks)
% ṡ = ks*es + Vχ*cos(eχ)
s_dot = ks * es + V_chi * cos(echi);
```

### 5.4 控制增益（论文默认值）

| 参数 | 符号 | 默认值 | 说明 |
|------|------|--------|------|
| 沿迹增益 | ks | 0.2 | 控制虚拟目标沿路径移动 |
| 航向增益 | kω | 0.005 | 航向误差反馈增益 |
| γ | gamma | 4000 | 误差权重 |
| k | k | 0.01 | approach angle参数 |
| χ∞ | chi_inf | π/2 | approach angle极限 |

---

## 6. 输出接口

### 6.1 Guidance输出

| 输出 | 符号 | 单位 | 说明 |
|------|------|------|------|
| 期望航向率 | ωd | rad/s | 论文Eq.6a的输出 |
| 当前航段索引 | leg_index | - | 来自RNP AR飞行计划 |
| 弧长变化率 | ṡ | m/s | 论文Eq.6b |

### 6.2 传递给后续模块
- ωd → 输入给现有内环控制（由你的AFCSModel处理）

---

## 7. 模块结构

```
┌─────────────────────────────────────────────────────────────┐
│  输入: AFCSModel的21个输出                                   │
│  - V, phi, psi, Latitude, Longitude, Chi                    │
├─────────────────────────────────────────────────────────────┤
│  模块1: 状态提取                                             │
│  - 从21个输出提取Vχ, χ, φ, 位置                              │
│  - 单位转换 (deg → rad)                                      │
├─────────────────────────────────────────────────────────────┤
│  模块2: S-F误差计算                                          │
│  - 加载RNP AR飞行计划                                        │
│  - 计算q(s), χf(s), κ(s)                                    │
│  - 计算es, ed, eχ                                           │
├─────────────────────────────────────────────────────────────┤
│  模块3: Kinematic Control                                    │
│  - 计算approach angle δ                                      │
│  - 计算期望航向率 ωd                                         │
│  - 计算弧长变化率 ṡ                                          │
├─────────────────────────────────────────────────────────────┤
│  输出: ωd (期望航向率)                                       │
└─────────────────────────────────────────────────────────────┘
```

---

## 8. 与现有代码的接口

### 8.1 复用函数

| 函数 | 用途 |
|------|------|
| `Manuscript_RNPAR_FlightPlan.m` | 加载RNP AR飞行计划 |
| `func_ApproachGuidance_RNPAR.m` | 航段解析（q, χf, κ计算） |
| `func_GreatCircleInverse.m` | 大圆距离/方位角计算 |

### 8.2 新增/修改文件

1. **新增**: `system_model_interface.m` - 系统模型接口
2. **新增**: `kinematic_guidance.m` - kinematic control law实现
3. **修改**: `Start.m` - 集成新的guidance模块

---

## 9. 验证方法

1. **数值仿真**: 使用论文中的圆形路径测试
2. **对比验证**: 与论文Fig.2-4的误差曲线对比
3. **HILS测试**: 接入现有的Simulink环境

---

## 10. 待定事项 (TBD)

- [ ] 实际飞行测试参数微调
- [ ] bounded bank angle限制的具体实现
- [ ] 与现有AFCSModel的集成方式确认