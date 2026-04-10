# Y8ModelAllControl 工程知识库（给 AI 使用）

本文档用于回答三个问题：
1. 这个文件夹里每个程序在做什么。
2. 飞机模型整体是什么结构。
3. 以后让 AI 帮你分析/改代码时，最少要提供哪些上下文。

文档基于当前工作区源码整理，重点覆盖可执行程序（.m / .mdl / .c）与核心数据接口。

## 1. 工程目标（一句话）

本工程是一个基于 MATLAB + Simulink 的 Y-8 飞机自动飞行控制与 RNP AR 进近仿真系统：
- 用飞行计划生成参考路径（TF/RF 航段）
- 用系统模型维护虚拟目标 q(s) 与误差
- 用制导/控制模块产生控制指令并驱动飞机动力学模型
- 输出轨迹、误差和关键监测量用于验证收敛性与性能

## 2. 目录中的“程序类”文件清单

### 2.1 MATLAB 脚本与函数（18 个）

#### 入口与初始化
- Start.m
  - 主入口脚本。
  - 做四件事：加载气动数据、做配平、加载飞行计划、按需编译 Angle_chi.c 生成 MEX。
  - 还会设置风场参数（wind_type/wind_direction/wind_speed）。

- AirCraftData.m
  - 飞机气动数据库与查表参数库。
  - 定义大量气动导数、系数表与发动机相关数据（并加载 thrust_data、ETA_data、fuel_data）。

- AircraftTrim.m
  - 配平脚本。
  - 根据质量 mass 选择 Inertia_Calculate_49/54/58/61/65.mdl 计算惯量，随后在 Aero_Dynamics 上调用 findop 求配平工作点。

#### 系统模型（虚拟目标与误差）
- system_model_init.m
  - 初始化系统模型结构体 sys_model。
  - 设置控制参数（k, ks, k_omega, gamma, chi_inf）、初始化 q(s)、航段索引与路径几何量。

- system_model_update.m
  - 每个仿真步更新 sys_model。
  - 负责：累计航程、航段同步、虚拟目标更新、误差计算、s 的不动点迭代推进。
  - 输出 es/ed/echi/s/chi_f/kappa/s_dot/q_lat/q_lon。

- func_SystemModelMonitorBlock.m
  - Simulink 监测支路包装函数。
  - 内部调用 system_model_init + system_model_update + func_KinematicControlLaw。
  - 输出 11 维向量，供模型记录到 SM_vector。

- func_KinematicControlLaw.m
  - 运动学控制律实现。
  - 输入 es/ed/echi/V/kappa 与参数，输出期望航向角速度 omega_d 和弧长变化率 s_dot。

#### RNP AR 制导与路径参数化
- func_ApproachGuidance_RNPAR.m
  - RNP AR 制导主函数（基于飞行计划、飞机状态、当前航段索引等）。
  - 输出当前航段信息与指导参数（横航偏差、航向、滚转、高度、垂直速度）。
  - 注意：该文件内部还定义了本地版本的 GreatCircleInverse/GreatCircleForward/CalculateArcAngle（与全局同名函数重复实现）。

- func_PathParameterization.m
  - 飞行计划路径参数化函数（TF/RF 统一输出 leg_params 结构）。
  - 当前在主链路中未被直接调用，属于可复用辅助层。

#### 几何计算函数族
- func_GreatCircleInverse.m
  - 大圆反算：两点 -> 距离 + 初始方位角。

- func_GreatCircleForward.m
  - 大圆正算：起点 + 方位角 + 弧长 -> 终点。

- func_RhumbLineInverse.m
  - 恒向线反算（WGS-84）。

- func_RhumbLineForward.m
  - 恒向线正算（WGS-84）。

- func_CalculateArcAngle.m
  - 圆弧角度计算：根据起终点方位角与转弯方向计算 RF 圆弧角。

#### 飞行计划与可视化
- Manuscript_RNPAR_FlightPlan.m
  - 定义测试飞行计划 RNPAR_FlightPlan（九寨黄龙场景，含 TF/RF 航段）。

- Manuscript_RNPAR_Figure.m
  - 绘制飞行计划参考航迹（水平+垂直），并叠加实际轨迹变量。

- plot_afcs_system_model_results.m
  - 面向系统模型监测输出的结果分析图。
  - 读取 SM_vector 与飞机轨迹日志，绘制 q(s)、误差、航段切换、chi_f/kappa 等。

- func_DrawAircraftModel.m
  - 飞机三维模型动画绘制函数（姿态/位置可视化）。

### 2.2 C / MEX（1 个）

- Angle_chi.c
  - Simulink S-Function。
  - 输入地速分量（VE、VN），输出航向角，供模型中的 S-Function 块调用。
  - Start.m 会按需编译得到 Angle_chi.mexw64。

### 2.3 Simulink 模型（核心）

- AFCSModel.mdl
  - 主系统模型（当前版本已保存为 R2025b OPC 文本包格式）。
  - 模型中确认存在：
    - MATLAB Function: func_SystemModelMonitorBlock(...)
    - MATLAB Function: func_ApproachGuidance_RNPAR
    - 记录变量：SM_vector、Plane_Lati、Plane_Longi 等
    - 子系统：Vertical_controll、backSteppingControl、backSteppingControlTanh、ndiControl
    - S-Function: Angle_chi

- Aero_Dynamics.mdl
  - 飞机动力学/气动主模型（传统 mdl 文本格式，编码 GBK）。
  - 根输入：delta_a / delta_e / delta_r / delta_p
  - 根输出：V, Alpha, Beta, p, q, r, phi, theta, psi, x, y, Z
  - 可见 Force/Moment 等内部子系统结构。

- Inertia_Calculate_49.mdl
- Inertia_Calculate_54.mdl
- Inertia_Calculate_58.mdl
- Inertia_Calculate_61.mdl
- Inertia_Calculate_65.mdl
  - 各质量档位惯量计算模型。
  - 输出并写入工作区变量 Ix、Iy、Iz、Ixz，供 AircraftTrim.m 使用。

### 2.4 模型历史版本（参考）
- AFCSModel.mdl.r2014a
- AFCSModel.mdl.r2023b
  - 历史兼容版本，不是当前主运行入口。

## 3. 飞机模型“是什么样的”

可以把它理解成四层：

1) 物理层（动力学/气动）
- Aero_Dynamics.mdl 负责状态演化和气动力/力矩相关计算。
- 关键状态输出：V, Alpha, Beta, p, q, r, phi, theta, psi, x, y, Z。

2) 惯量层（质量档位）
- Inertia_Calculate_*.mdl 根据质量档位输出 Ix/Iy/Iz/Ixz。
- AircraftTrim.m 按 mass 自动选对应模型，并完成配平工作点求解。

3) 制导层（路径/误差）
- func_ApproachGuidance_RNPAR.m 负责 RNP AR 航段级制导参数。
- system_model_init.m + system_model_update.m 负责虚拟目标 q(s) 与误差动力学。

4) 控制层（航向/滚转/纵向）
- AFCSModel.mdl 中的 Vertical_controll、backSteppingControl、backSteppingControlTanh、ndiControl 等子系统。
- 角度处理依赖 Angle_chi S-Function。

## 4. 主执行链路（从启动到结果）

1. 运行 Start.m
- 加载 AirCraftData
- 执行 AircraftTrim
- 加载 Manuscript_RNPAR_FlightPlan
- 按需编译 Angle_chi.c

2. 启动 AFCSModel.mdl 仿真
- 模型内部调用 func_ApproachGuidance_RNPAR
- 模型内部调用 func_SystemModelMonitorBlock

3. 监测输出
- SM_vector 被记录为 Timeseries
- Plane_Lati / Plane_Longi / Plane_Height 等变量被记录到工作区

4. 结果分析
- 运行 plot_afcs_system_model_results.m 或 Manuscript_RNPAR_Figure.m

## 5. 核心数据接口（AI 分析最重要）

### 5.1 飞行计划矩阵 RNPAR_FlightPlan
- 维度：N x 9
- 列定义：
  1) leg_type（0=IF, 1=TF, 2=RF）
  2) lat（deg）
  3) lon（deg）
  4) alt（m）
  5) turn_dir（-1 左转, +1 右转）
  6) radius（m）
  7) center_lat（deg）
  8) center_lon（deg）
  9) RNP（m）

### 5.2 监测输出 SM_vector（11 维）
顺序为：
1) q_lat
2) q_lon
3) es
4) ed
5) echi
6) s
7) leg_index
8) chi_f
9) kappa
10) s_dot
11) omega_d

### 5.3 飞机状态输入（system_model_update 内部约定）
aircraft_state = [lat, lon, chi_deg, V, ...]

## 6. 函数关系图（简化）

Start.m
-> AirCraftData.m
-> AircraftTrim.m -> Inertia_Calculate_*.mdl
-> Manuscript_RNPAR_FlightPlan.m
-> AFCSModel.mdl
   -> func_ApproachGuidance_RNPAR.m
   -> func_SystemModelMonitorBlock.m
      -> system_model_init.m
      -> system_model_update.m
         -> func_GreatCircleInverse/Forward
         -> func_RhumbLineInverse
         -> func_CalculateArcAngle
      -> func_KinematicControlLaw.m
-> plot_afcs_system_model_results.m

## 7. 当前已知技术风险与注意事项

1) 同名函数重复实现（高优先级）
- func_ApproachGuidance_RNPAR.m 内部定义了：
  - func_GreatCircleInverse
  - func_GreatCircleForward
  - func_CalculateArcAngle
- 工程同时存在同名独立文件。
- 风险：后续维护时容易出现“改了全局函数但局部函数没改”的行为偏差。

2) 文档与代码参数可能不一致
- docs/system_model_implementation_explanation.md 中部分参数值与当前 system_model_init.m 不一致。
- 结论：做分析时请以当前源码为准。

3) 编码混合（GBK/UTF-8）
- 多个历史 matlab 文件含 GBK 中文注释。
- 注意避免批量转码导致无关 diff。

4) MEX 文件占用问题
- Angle_chi.mexw64 可能被 MATLAB 进程占用导致重编译失败。
- Start.m 已做“按需编译 + clear('Angle_chi')”防护。

5) 兼容文件状态
- 旧拼写函数 func_GreatCincleForward.m 已移除。
- 当前主代码调用已统一为 func_GreatCircleForward。

6) 文件存在性差异
- 当前源码清单中不存在 func_OmegaDOnlyBlock.m。
- 如果旧模型或旧脚本引用该文件，需要补回或改线。

## 8. 给 AI 提问时建议附带的最小上下文

每次提问建议至少给以下信息：

1. 你要分析的目标
- 例如：发散原因、误差过大原因、某航段切换异常、重构命名、改控制参数。

2. 仿真入口与时长
- 是否通过 Start.m 启动。
- 仿真 StopTime 和关键工况（mass、Vt、alt、wind_type）。

3. 必要变量
- RNPAR_FlightPlan
- SM_vector（至少前 10 列）
- Plane_Lati / Plane_Longi / Plane_Height（如需轨迹分析）

4. 你观察到的指标
- max|es|, max|ed|, max|echi|, 是否收敛、是否振荡。

5. 你是否改过这些文件
- system_model_init.m
- system_model_update.m
- func_KinematicControlLaw.m
- func_ApproachGuidance_RNPAR.m

## 9. 可直接复用的 AI 提问模板

### 模板 A：排查不收敛
请基于 Start.m + AFCSModel.mdl + system_model_update.m + func_KinematicControlLaw.m 帮我定位不收敛原因。
已知条件：mass=54000, Vt=80, StopTime=40。
指标：max|ed|=XX, max|echi|=XX。
请区分是参数问题、模型连线问题还是几何计算问题。

### 模板 B：做最小改动修复
请只修改 system_model_update.m，目标是减少 q(s) 抖动，保持接口不变。
先说明改动点，再给出 patch，不要改其他文件。

### 模板 C：做命名/结构重构
请重构几何函数调用链，去除 func_ApproachGuidance_RNPAR.m 内部与全局重复函数。
要求：行为不变、增加回归检查脚本、列出风险。

## 10. 后续维护建议

1. 新增一个 docs/geometry_conventions.md
- 统一度/弧度、左/右转符号、ed 正负方向定义。

2. 把 func_ApproachGuidance_RNPAR.m 内部重复函数外提
- 减少同名重复维护风险。

3. 给 system_model_init.m / system_model_update.m 加版本注释
- 明确参数版本和调参日期。

4. 建议补一份最小回归脚本
- 输入固定 flight plan
- 检查 SM_vector 维度、q(s) 连续性、误差上界

---

如果要把这份文档当成 AI 的长期上下文，建议每次结构性改动后只更新三块内容：
- 文件清单变化
- 主执行链路变化
- 数据接口变化
