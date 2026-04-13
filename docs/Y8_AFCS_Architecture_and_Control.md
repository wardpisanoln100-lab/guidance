# Y-8 飞机 RNP AR 进场制导与飞控系统 — 架构与控制方法说明

> 编写日期: 2026-04-13
> 适用项目: y8ModelAllControl (运-8 型飞机 RNP AR 进场制导 Simulink 仿真)

---

## 目录

1. [项目概述](#1-项目概述)
2. [飞机模型](#2-飞机模型)
3. [RNP AR 进场飞行计划](#3-rnp-ar-进场飞行计划)
4. [整体控制架构](#4-整体控制架构)
5. [系统模型: Serret-Frenet 虚拟目标跟踪](#5-系统模型-serret-frenet-虚拟目标跟踪)
6. [运动学控制律 (论文 Eq.5, 6a, 6b)](#6-运动学控制律-论文-eq5-6a-6b)
7. [AFCS 飞控模型](#7-afcs-飞控模型)
8. [文件结构与功能](#8-文件结构与功能)
9. [信号流与数据流](#9-信号流与数据流)
10. [坐标系统与导航函数](#10-坐标系统与导航函数)
11. [关键参数表](#11-关键参数表)
12. [仿真流程](#12-仿真流程)

---

## 1. 项目概述

本项目为**运-8 (Y-8) 大型运输机**的 **RNP AR (Required Navigation Performance - Authorization Required)** 进场制导与飞行控制系统仿真。

### 1.1 应用场景

在**九寨黄龙机场**执行 RNP AR 精密进场程序。RNP AR 是一种高性能的基于性能的导航 (PBN) 程序，要求：

- 航迹容差 ≤ 0.3 NM
- 支持复杂地形条件下的固定半径转弯 (RF 航段)
- 需要机载性能监测与告警 (OPMA)

### 1.2 技术路线

采用**基于论文的路径跟随 (Path Following)** 方法，核心为:

1. **Serret-Frenet (S-F) 坐标系**下的虚拟目标跟踪
2. **非线性运动学控制律**生成期望航向角速度 `omega_d`
3. **协调转弯关系**将 `omega_d` 转换为滚转角指令 `phi_c`
4. **AFCS 内部滚转/偏航控制**将 `phi_c` 转为舵面偏转指令

### 1.3 参考论文

Kim, S.; Jung, D. "Adaptive Path Guidance Law for a Small Fixed-Wing UAS with Bounded Bank Angle." *Drones* 2025, 9, 180.

> 本项目实现了论文中的**运动学控制律**部分 (Section 2.2.1, Eq.5, 6a, 6b)，即路径跟随的核心算法。论文中的反步法层 (Section 2.2.2) 不适用于 Y-8 这类大型慢响应飞机，因此未采用。

---

## 2. 飞机模型

### 2.1 飞机参数

| 参数 | 数值 | 说明 |
|------|------|------|
| 机型 | 运-8 (Y-8) | 中型四发涡轮螺旋桨运输机 |
| 质量 | 54,000 kg | 仿真配平质量 |
| 进场速度 | 80 m/s | 约 156 节 |
| 进场高度 | 1284.73 m | 九寨黄龙机场标高 |
| 初始航向 | 15.95° | 进场航迹角 |

### 2.2 动力学特点

Y-8 作为大型运输机，具有以下关键动力学特征：

| 特征 | 数值/范围 | 影响 |
|------|-----------|------|
| 滚转响应时间 | 0.3-0.5 秒 | 滚转指令到实际滚转的滞后 |
| 最大滚转率 | ~5-8°/秒 | 转弯速率受限 |
| 最大滚转角 | ±30° | 进场程序限制 |
| 惯量大 | — | 对控制指令响应迟缓 |

这些特征直接影响了控制律的选择：论文中的反步法层假设飞机能快速精确跟踪滚转指令，这一假设对 Y-8 **不成立**，因此本项目仅采用运动学控制律 + 协调转弯转换的简化方案。

---

## 3. RNP AR 进场飞行计划

### 3.1 航路点定义

飞行计划定义在 [Manuscript_RNPAR_FlightPlan.m](../Manuscript_RNPAR_FlightPlan.m) 中，包含 6 个航路点，5 个航段：

| 序号 | 航段类型 | 纬度 (°) | 经度 (°) | 高度 (m) | 转弯 | 半径 (m) | RNP (NM) |
|------|----------|----------|----------|----------|------|----------|----------|
| IF | — | 32.6261 | 103.5940 | 1284.73 | — | — | 0.3 |
| TF | — | 32.6693 | 103.6087 | 1284.73 | — | — | 0.3 |
| RF | 左 | 32.7370 | 103.6286 | 922.63 | L | 31750 | 0.3 |
| RF | 右 | 32.8023 | 103.6603 | 503.83 | R | 14808 | 0.3 |
| RF | 左 | 32.8202 | 103.6709 | 384.05 | L | 5926 | 0.3 |
| TF | — | 32.8661 | 103.6865 | 106.70 | — | — | 0.3 |

### 3.2 航段类型

- **IF (Initial Fix)**: 初始定位点，程序起始位置
- **TF (Track to Fix)**: 大圆弧直线航段
- **RF (Radius to Fix)**: 固定半径转弯航段（RNP AR 的核心特征）

### 3.3 航段几何

```
IF ──(TF, 直线)──> 第一个航路点
                    │
                    │ RF 左转 (r=31750m)
                    ▼
               第二个航路点
                    │
                    │ RF 右转 (r=14808m)
                    ▼
               第三个航路点
                    │
                    │ RF 左转 (r=5926m)
                    ▼
               跑道入口 (TF)
```

RNP 值 0.3 NM 表示航迹容差为 ±0.3 海里 (约 ±556 米)。

---

## 4. 整体控制架构

### 4.1 分层结构

```
┌──────────────────────────────────────────────────────────┐
│                    MATLAB/Simulink 仿真环境               │
│                                                          │
│  ┌────────────────────────────────────────────────────┐  │
│  │            制导层 (Guidance Layer)                  │  │
│  │                                                    │  │
│  │  ┌──────────────┐    ┌──────────────────────────┐  │  │
│  │  │ 飞行计划     │───>│ 系统模型 (Serret-Frenet) │  │  │
│  │  │ (Waypoints)  │    │ - 虚拟目标 q(s)           │  │  │
│  │  │              │    │ - 跟踪误差 es, ed, echi   │  │  │
│  │  └──────────────┘    └────────────┬─────────────┘  │  │
│  │                                   │                 │  │
│  │                    ┌──────────────▼──────────────┐  │  │
│  │                    │ 运动学控制律                │  │  │
│  │                    │ - Eq.5: 进场角 delta(ed)    │  │  │
│  │                    │ - Eq.6a: omega_d (期望航向) │  │  │
│  │                    │ - Eq.6b: s_dot (推进速度)   │  │  │
│  │                    └──────────────┬──────────────┘  │  │
│  │                                   │                 │  │
│  │                    ┌──────────────▼──────────────┐  │  │
│  │                    │ 滚转指令转换                │  │  │
│  │                    │ phi_c = atan(V*omega_d/g)   │  │  │
│  │                    └──────────────┬──────────────┘  │  │
│  └───────────────────────────────────│─────────────────┘  │
│                                      │                    │
│  ┌───────────────────────────────────│─────────────────┐  │
│  │            飞控层 (AFCS Layer)    │                  │  │
│  │                                   │                  │  │
│  │                    ┌──────────────▼──────────────┐  │  │
│  │                    │ Roll_modify 滚转控制        │  │  │
│  │                    │ - phi_c → delta_a (副翼)    │  │  │
│  │                    │ - phi_c → delta_r (方向舵)  │  │  │
│  │                    └──────────────┬──────────────┘  │  │
│  │                                   │                 │  │
│  │                    ┌──────────────▼──────────────┐  │  │
│  │                    │ 飞机动力学模型               │  │  │
│  │                    │ - 6DOF 飞行动力学            │  │  │
│  │                    │ - 气动数据表 (AirCraftData)  │  │  │
│  │                    │ - 发动机推力模型             │  │  │
│  │                    └─────────────────────────────┘  │  │
│  └────────────────────────────────────────────────────┘  │
│                                                          │
│  ┌────────────────────────────────────────────────────┐  │
│  │            监控与可视化层                           │  │
│  │  - func_SystemModelMonitorBlock (Simulink 封装)    │  │
│  │  - plot_afcs_system_model_results (MATLAB 绘图)    │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
```

### 4.2 制导 vs 飞控的分工

| 层 | 职责 | 输入 | 输出 |
|----|------|------|------|
| 制导层 | 计算"飞机应该往哪里飞" | 飞机位置、航向、速度 + 飞行计划 | 滚转角指令 `phi_c` (rad) |
| 飞控层 | 计算"舵面怎么偏才能让飞机滚到目标角度" | `phi_c` + 飞机状态 | 副翼偏转 `delta_a`、方向舵偏转 `delta_r` |

这是一个**典型的制导-控制分离架构**：制导层只管"目标方向"，飞控层只管"如何到达"。

---

## 5. 系统模型: Serret-Frenet 虚拟目标跟踪

### 5.1 核心概念

传统的轨迹跟踪要求飞机在**指定时刻**到达**指定位置**。而路径跟随只要求飞机**靠近参考路径**，不关心时间同步。

本项目采用**虚拟目标 (Virtual Target)** 方法：

```
参考路径: ───────────────────────────>
           q(s) ← 虚拟目标在路径上移动
                   │
                   │ ed (横向误差)
                   │
              飞机位置
                   │
                   │ es (纵向误差)
                   ▼
              路径切向
```

### 5.2 Serret-Frenet (S-F) 坐标系

在参考路径上的点 q(s) 处建立局部坐标系：

- **x 轴**: 路径切向方向 (与航向 `chi_f` 一致)
- **y 轴**: 路径法向方向 (指向路径左侧)

飞机相对于 q(s) 的误差分解为：

| 误差 | 符号 | 物理含义 | 正方向 |
|------|------|----------|--------|
| 纵向误差 | `es` | 飞机沿路径方向偏离 q(s) 的距离 | 飞机在 q(s) 前方为正 |
| 横向误差 | `ed` | 飞机垂直于路径方向的偏离距离 | 飞机在路径右侧为正 |
| 航向误差 | `echi` | 飞机航向与路径切向的夹角 | `chi - chi_f` |

### 5.3 误差动力学方程

S-F 坐标系下的误差动力学（论文 Eq.4a-4c）：

```
ė_s  = V_χ cos(χ_e) - (1 - κ(s)·e_d)·ṡ        (4a)
ė_d  = V_χ sin(χ_e) - κ(s)·e_s·ṡ              (4b)
χ̇_e  = ω - κ(s)·ṡ                              (4c)
```

其中：
- `V_χ`: 地速 (m/s)
- `χ_e = χ - χ_f`: 航向误差
- `κ(s)`: 路径曲率 (直线段为 0，圆弧段为 ±1/R)
- `ṡ = s_dot`: 虚拟目标沿路径的推进速度

### 5.4 虚拟目标推进策略

虚拟目标的推进速度 `s_dot` 是一个**控制自由度**：

```
s_dot = k_s · e_s + V_χ · cos(χ_e)           (Eq.6b)
```

- `k_s · e_s`: 当飞机在虚拟目标前方 (es > 0) 时加速推进，缩小纵向误差
- `V_χ · cos(χ_e)`: 飞机速度在路径切向的投影

这相当于让虚拟目标"追着飞机跑"，而不是以固定速度前进。

### 5.5 实现文件

- [system_model_update.m](../system_model_update.m): 计算误差 `es`, `ed`, `echi` 和推进速度 `s_dot`
  - `compute_tracking_errors()`: S-F 坐标系误差计算
  - `update_virtual_target_state()`: 用不动点迭代更新虚拟目标位置
  - `compute_virtual_target()`: 根据航段类型（TF/RF）计算 q(s) 的位置和几何量
- [system_model_init.m](../system_model_init.m): 初始化系统模型状态和参数

---

## 6. 运动学控制律 (论文 Eq.5, 6a, 6b)

### 6.1 进场角函数 (Eq.5)

```
δ(e_d) = -χ_∞ · (e^(2k·e_d) - 1) / (e^(2k·e_d) + 1)
```

**作用**: 根据横向误差 `ed` 计算一个期望的"进场角"。

**特性**:
- 这是双曲正切函数的变体: `δ(e_d) ≈ -χ_∞ · tanh(k·e_d)`
- `ed → +∞` 时, `δ → -χ_∞` (飞机在右侧，进场角向左偏)
- `ed → -∞` 时, `δ → +χ_∞` (飞机在左侧，进场角向右偏)
- `ed = 0` 时, `δ = 0` (在路径上，进场角为 0)

**参数**:
- `χ_∞ = π/2`: 最大进场角（90°）
- `k = 0.002`: 进场角变化的陡峭程度

### 6.2 期望航向角速度 (Eq.6a)

```
ω_d = -k_ω(χ_e - δ) + κ(s)·ṡ
      + δ'(e_d)[V_χ sin(χ_e) - κ(s)·e_s·ṡ]
      - (e_d·V_χ/γ)·sinc((χ_e-δ)/2)·cos((χ_e+δ)/2)
```

**四项的物理含义**:

| 项 | 表达式 | 物理含义 |
|----|--------|----------|
| 航向反馈 | `-k_ω(χ_e - δ)` | 将当前航向误差拉向进场角方向 |
| 曲率补偿 | `κ(s)·ṡ` | 在弯道上需要的额外转弯率 |
| 进场角变化 | `δ'·[V_χ sin(χ_e) - κ·e_s·ṡ]` | 考虑进场角随位置的变化率 |
| 横向收敛 | `-(e_d·V_χ/γ)·sinc·cos` | 驱动横向误差归零的项 |

### 6.3 推进速度 (Eq.6b)

```
ṡ = k_s · e_s + V_χ · cos(χ_e)
```

详见 [5.4 节](#54-虚拟目标推进策略)。

### 6.4 稳定性

该控制律的稳定性在论文参考文献 [26] 中有完整的 Lyapunov 证明。核心要点：

- 控制律保证 `e_s → 0`, `e_d → 0`, `χ_e → 0` 当 `t → ∞`
- 增益条件: `k_s > 0`, `k_ω > 0`, `γ > 0`, `k > 0`, `χ_∞ ∈ (0, π/2)`

### 6.5 实现文件

[func_KinematicControlLaw.m](../func_KinematicControlLaw.m):

```matlab
function [omega_d, s_dot, detail] = func_KinematicControlLaw(es, ed, echi, V_chi, kappa, params)
```

---

## 7. AFCS 飞控模型

### 7.1 Simulink 模型结构

`AFCSModel.mdl` 是整个系统的 Simulink 仿真模型，包含：

```
AFCSModel
├── Aircraft 6DOF (飞机六自由度动力学)
├── RNP AR Approach Guidance (制导模块)
│   ├── func_SystemModelMonitorBlock (系统模型监测)
│   ├── func_ApproachGuidance_RNPAR (备用 RNPAR 制导)
│   └── 坐标转换与信号处理
├── Roll_modify (滚转控制)
├── Lateral_controll (侧向控制)
├── Longitudinal_controll (纵向控制)
└── 环境模块 (风场、大气模型)
```

### 7.2 制导信号流

```
MonitorBlock (func_SystemModelMonitorBlock)
    ↓ [11 路输出]
Selector → y(11) = omega_d
    ↓
Product × V_aircraft → Product / g → atan → Gain×57.3
    ↓
Scope (仅显示，不用于控制)

MonitorBlock
    ↓ [11 路输出]
Selector → y(11) = omega_d
    ↓
计算 phi_d = atan(V × omega_d / g)   ← 协调转弯转换
    ↓
To Workspace (SM_vector)
    ↓
Goto "RNPAR_phi_guidance"
    ↓
Roll_modify subsystem
    ├── 限幅 ±30°
    ├── Sum3: phi_g - phi_actual (滚转误差)
    ├── Gain → Sum1 → delta_a (副翼)
    ├── Gain4, Gain3 → Sum2 → delta_r (方向舵)
    └── 输出到飞机动力学模型
```

### 7.3 协调转弯转换

将期望航向角速度转换为滚转角的核心公式（论文 Eq.7）：

```
φ_d = atan(V_χ · ω_d / g)
```

**推导**: 在无侧滑协调转弯中，向心加速度由重力的水平分量提供：

```
tan(φ) = (V²/R) / g = (V · ω) / g
```

其中 `ω = V/R` 是转弯角速度（即航向角速度 `omega_d`）。

这个转换**不包含任何反馈修正**，只是一个前馈计算。实际的滚转跟踪由 AFCS 内部的 `Roll_modify` 完成。

### 7.4 Roll_modify 子系统

```
输入: phi_g (滚转指令, 来自制导层)
      phi_actual (实际滚转角, 来自飞机模型)

处理:
  1. 限幅: phi_cmd = saturate(phi_g, ±30°)
  2. 误差: e_phi = phi_cmd - phi_actual
  3. 副翼: delta_a = K1 × e_phi
  4. 方向舵: delta_r = K2 × e_phi + K3 × r (偏航率)

输出: delta_a (副翼偏转角), delta_r (方向舵偏转角)
```

这是一个**比例控制**的滚转角跟踪回路，结构简洁。

---

## 8. 文件结构与功能

### 8.1 入口与初始化

| 文件 | 功能 |
|------|------|
| `Start.m` | 仿真入口。加载飞机数据、配平计算、加载飞行计划、编译 MEX 文件、设置风场 |
| `system_model_init.m` | 初始化系统模型结构体。设置控制参数、飞行计划、虚拟目标初始状态 |

### 8.2 系统模型（制导核心）

| 文件 | 功能 |
|------|------|
| `system_model_update.m` | 每个仿真步调用。计算 S-F 误差、更新虚拟目标位置、返回 `s_dot` |
| `func_SystemModelMonitorBlock.m` | Simulink S-Function 包装。调用 init/update/KinematicControlLaw，输出 11 路信号 |
| `func_KinematicControlLaw.m` | 运动学控制律核心实现 (Eq.5, 6a, 6b) |

### 8.3 飞行计划与导航

| 文件 | 功能 |
|------|------|
| `Manuscript_RNPAR_FlightPlan.m` | 定义 RNP AR 进场飞行计划（6 个航路点） |
| `func_PathParameterization.m` | 将飞行计划转换为路径参数化格式 |
| `func_GreatCircleInverse.m` | 大圆反算: 两点间距离和方位角 |
| `func_GreatCircleForward.m` | 大圆正算: 起点+方位角+距离 → 终点 |
| `func_RhumbLineInverse.m` | 恒向线反算（用于圆弧航段） |
| `func_CalculateArcAngle.m` | 计算 RF 航段的圆弧角度 |

### 8.4 飞机模型

| 文件 | 功能 |
|------|------|
| `AirCraftData.m` | 飞机气动数据表。包含升力系数、阻力系数、力矩系数等查表数据 |
| `AircraftTrim.m` | 配平计算。给定速度和高度，计算配平状态（俯仰角、舵面偏转等） |

### 8.5 备用制导

| 文件 | 功能 |
|------|------|
| `func_ApproachGuidance_RNPAR.m` | 传统 RNPAR 制导算法（备用方案，计算 XTK, chi_g, phi_g, H_g, Vs_g） |

### 8.6 可视化

| 文件 | 功能 |
|------|------|
| `plot_afcs_system_model_results.m` | 绘制仿真结果。包含轨迹图和 12 个子图（误差、控制量等） |
| `func_DrawAircraftModel.m` | 3D 飞机模型可视化 |
| `Manuscript_RNPAR_Figure.m` | 论文配图生成 |

### 8.7 辅助函数

| 文件 | 功能 |
|------|------|
| `func_RhumbLineForward.m` | 恒向线正算 |

---

## 9. 信号流与数据流

### 9.1 初始化流程

```
Start.m
  ├── AirCraftData → 加载气动数据表
  ├── AircraftTrim → 计算配平状态
  ├── Manuscript_RNPAR_FlightPlan → 加载飞行计划
  └── 编译 MEX 文件 (Angle_chi.c)

AFCSModel (Simulink)
  ├── func_SystemModelMonitorBlock (首次调用)
  │   └── system_model_init(flight_plan)
  │       ├── 初始化 sys_model 结构体
  │       ├── 设置控制参数 (k, ks, k_omega, gamma, chi_inf)
  │       └── 计算初始路径几何量
  └── 仿真开始...
```

### 9.2 每步仿真数据流

```
每个仿真步 (dt):

1. 飞机状态 → MonitorBlock 输入:
   aircraft_lat, aircraft_lon, aircraft_chi_rad, aircraft_V

2. MonitorBlock 内部:
   system_model_update() → errors (es, ed, echi, s, leg_index, chi_f, kappa, s_dot)
       │
       └── func_KinematicControlLaw(errors, V_chi, kappa)
           ├── Eq.5: delta(ed)
           ├── Eq.6a: omega_d
           └── Eq.6b: s_dot (与 update 中计算的一致)

3. MonitorBlock 输出 (11 路):
   [q_lat, q_lon, es, ed, echi, s, leg_index, chi_f, kappa, s_dot, omega_d]

4. Simulink 后续处理:
   omega_d → atan(V*omega_d/g) → phi_d → RNPAR_phi_guidance → Roll_modify → delta_a, delta_r

5. delta_a, delta_r → AFCS → 飞机动力学 → 新的飞机状态
```

### 9.3 监控信号输出

Simulink 的 To Workspace 模块将 MonitorBlock 输出记录为 `SM_vector`，供 `plot_afcs_system_model_results.m` 读取绘图：

| 索引 | 信号 | 单位 | 含义 |
|------|------|------|------|
| 1 | q_lat | 度 | 虚拟目标纬度 |
| 2 | q_lon | 度 | 虚拟目标经度 |
| 3 | es | m | 纵向误差 |
| 4 | ed | m | 横向误差 |
| 5 | echi | rad | 航向误差 |
| 6 | s | m | 路径坐标 |
| 7 | leg_index | — | 当前航段编号 |
| 8 | chi_f | rad | 路径切向角 |
| 9 | kappa | 1/m | 路径曲率 |
| 10 | s_dot | m/s | 虚拟目标推进速度 |
| 11 | omega_d | rad/s | 期望航向角速度 |

---

## 10. 坐标系统与导航函数

### 10.1 坐标系

系统使用多种坐标系：

| 坐标系 | 描述 | 用途 |
|--------|------|------|
| WGS-84 | 经纬度 (lat, lon) | 飞机位置、航路点定义 |
| S-F | Serret-Frenet (es, ed) | 跟踪误差计算 |
| NED | 北东地 (局部直角) | 飞机姿态、速度分解 |
| 机体 | 机体轴系 (x_body, y_body, z_body) | 气动力计算 |

### 10.2 大圆导航 (Great Circle)

TF 航段使用大圆弧线（球面上两点间最短路径）：

```
func_GreatCircleInverse(lat1, lon1, lat2, lon2)
  → [distance_m, bearing_deg]   (反算: 两点 → 距离+方位)

func_GreatCircleForward(lat, lon, bearing_deg, distance_m)
  → [new_lat, new_lon]          (正算: 起点+方位+距离 → 终点)
```

使用 WGS-84 椭球参数（R_earth = 6371000 m）。

### 10.3 恒向线导航 (Rhumb Line)

RF 航段的圆弧计算使用恒向线：

```
func_RhumbLineInverse(center_lat, center_lon, point_lat, point_lon)
  → [distance, bearing]  (从圆心到点的方位角)
```

恒向线保持方位角不变，在短距离上与大圆近似。

### 10.4 RF 航段几何

对于固定半径转弯：

```
圆心 C(center_lat, center_lon)
  │
  │ R (转弯半径)
  ▼
弧起点 S → 弧上任意点 q(s) → 弧终点 E

q(s) 的位置:
  bearing_c2q = bearing_c2s + turn_dir × (s_local / R)
  q = GreatCircleForward(center_lat, center_lon, bearing_c2q, R)

路径几何量:
  chi_f = bearing_c2q + turn_dir × 90°  (切向 = 径向 + 90°)
  kappa = turn_dir / R                   (曲率 = ±1/R)
```

---

## 11. 关键参数表

### 11.1 运动学控制律参数

定义在 [system_model_init.m](../system_model_init.m):

| 参数 | 符号 | 值 | 物理含义 | 论文方程 |
|------|------|---|----------|----------|
| k | k | 0.002 | 进场角变化速率参数 | Eq.5 |
| ks | k_s | 0.2 | 纵向误差增益 | Eq.6b |
| k_omega | k_ω | 0.12 | 航向角速度反馈增益 | Eq.6a |
| gamma | γ | 1,000,000 | 横向收敛增益分母 | Eq.6a |
| chi_inf | χ_∞ | π/2 (90°) | 最大进场角 | Eq.5 |

### 11.2 飞机参数

| 参数 | 符号 | 值 |
|------|------|---|
| 重力加速度 | g | 9.81 m/s² |
| 地球半径 | R_earth | 6,371,000 m |
| 飞机质量 | m | 54,000 kg |
| 进场速度 | V_χ | 80 m/s |

### 11.3 控制约束

| 约束 | 值 | 来源 |
|------|---|------|
| 最大滚转角 | ±30° (±π/6 rad) | Roll_modify 限幅 |
| RNP 容差 | 0.3 NM (≈556 m) | 飞行计划定义 |

### 11.4 参数调优指南

| 参数 | 增大效果 | 减小效果 | 风险 |
|------|----------|----------|------|
| k_ω | 航向收敛更快 | 收敛更慢 | 过大→震荡 |
| k_s | 纵向误差消除更快 | 更慢 | 影响 s_dot 平滑性 |
| k | 进场角过渡更陡 | 过渡更平缓 | 过大→控制量突变 |
| γ | 横向收敛项减弱 | 横向收敛增强 | 过小→过度修正 |

---

## 12. 仿真流程

### 12.1 启动仿真

```matlab
% Step 1: 初始化
Start.m
  % 加载飞机数据
  % 配平计算
  % 加载飞行计划
  % 编译 MEX

% Step 2: 打开并运行 Simulink 模型
open_system('AFCSModel')
sim('AFCSModel')

% Step 3: 查看结果
plot_afcs_system_model_results()
```

### 12.2 Simulink 模型

`AFCSModel.mdl` 包含以下主要子系统：

```
AFCSModel
├── system (飞机动力学)
│   ├── 6DOF 模型
│   ├── 气动模型 (使用 AirCraftData 查表)
│   ├── 发动机模型
│   └── 风场/大气模型
├── control (飞控)
│   ├── Roll_modify (滚转)
│   ├── Lateral_controll (侧向)
│   └── Longitudinal_controll (纵向)
└── RNP AR Approach Guidance (制导)
    ├── func_SystemModelMonitorBlock (核心制导)
    └── 信号路由与转换
```

### 12.3 数据记录

Simulink 记录以下变量到 MATLAB 基工作区：

| 变量 | 来源 | 用途 |
|------|------|------|
| `SM_vector` | MonitorBlock To Workspace | 制导信号时间序列 |
| `Sim_time` / `tout` | Simulink 求解器 | 时间向量 |
| `Plane_Lati` / `Latitude` | 飞机模型输出 | 飞机轨迹 |
| `Plane_Longi` / `Longitude` | 飞机模型输出 | 飞机轨迹 |
| `RNPAR_FlightPlan` | Manuscript_RNPAR_FlightPlan | 飞行计划 |

### 12.4 结果可视化

运行 `plot_afcs_system_model_results()` 生成两个图：

**图 1: AFCS Horizontal Geometry**
- 参考路径 (蓝色)
- 飞机轨迹 (红色)
- 虚拟目标 q(s) (绿色三角)
- 航路点 (蓝色五角星)
- RF 航段圆心 (蓝色圆圈)

**图 2: AFCS System Model Monitor** (12 子图)
1. 经纬度轨迹
2. 纵向误差 es
3. 横向误差 ed
4. 航向误差 echi
5. 虚拟目标推进速度 s_dot
6. 路径坐标 s
7. 切向角 chi_f
8. 路径曲率 kappa
9. 期望航向角速度 omega_d
10. 滚转指令 phi_c
11. 自适应时间常数估计 lambda
12. 航向角速度跟踪误差 omega_e

---

## 附录 A: 未采用的反步法说明

论文 Section 2.2.2-2.2.3 包含反步法设计和自适应参数估计，本项目未采用，原因：

### A.1 反步法的核心假设

反步法 Eq.11 的 Lyapunov 稳定性证明假设：

```
φ = φ_c  (实际滚转角 = 指令滚转角)
```

即飞机能**精确且快速**地跟踪滚转指令。

### A.2 Y-8 的实际动力学

| 指标 | 反步法假设 | Y-8 实际 | 结论 |
|------|-----------|----------|------|
| 滚转响应时间 | < 0.1 s | 0.3-0.5 s | 滞后 3-5 倍 |
| 最大滚转率 | ~30°/s | 5-8°/s | 慢 4-6 倍 |

### A.3 不匹配的后果

由于 `φ ≠ φ_c`，Lyapunov 证明中的一项变为正：

```
实际 V̇ = V_proof + (g/V)·[tan(φ) - tan(φ_c)]·...
```

导致 V 不再单调递减，稳定性无法保证。仿真中表现为：

- 自适应参数 `lambda_hat` 持续漂移
- `phi_c` 在 ±30° 间震荡
- `ed` 和 `echi` 发散

### A.4 采用简化方案的原因

```
omega_d → φ_d = atan(V·omega_d/g) → Roll_modify
```

这个方案：
- 只做前馈转换（协调转弯关系）
- 将滚转跟踪完全交给 AFCS 内部
- 避免了反步法的双反馈冲突
- 虽然理论上不够"完美"，但对 Y-8 的实际效果更好

---

## 附录 B: 常用 MATLAB 命令

```matlab
% 启动仿真
run('Start.m')
open_system('AFCSModel')
sim('AFCSModel')

% 查看结果
plot_afcs_system_model_results()

% 检查制导信号
disp(max(abs(evalin('base', 'SM_vector')(:,4))))  % max|ed|
disp(max(abs(evalin('base', 'SM_vector')(:,5))))  % max|echi|

% 清除工作区
clear all
close all
```
