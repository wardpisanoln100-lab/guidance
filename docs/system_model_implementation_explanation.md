# `system_model_init.m` 与 `system_model_update.m` 实现说明

本文档说明当前工程中这两个文件是如何把论文中的水平面系统模型落到代码里的。重点不是重新推导整篇论文，而是把下面几件事讲清楚：

- 代码里每个状态量代表什么
- 初始化阶段做了什么
- 每次更新阶段做了什么
- 每个辅助函数负责什么
- 代码和论文公式是怎样对应起来的

---

## 1. 这两个文件在整个工程里扮演什么角色

这两个文件实现的是“系统模型状态管理层”。

它们不直接输出最终控制指令，而是负责维护路径跟踪问题里最基础的几何和误差信息：

- 虚拟目标 `q(s)`
- 路径切向角 `chi_f`
- 路径曲率 `kappa`
- 沿迹误差 `es`
- 横迹误差 `ed`
- 航迹角误差 `echi`

后续 guidance law 要用的就是这些量。

从论文角度看，这里实现的是第 2.1 节里和路径几何、S-F 坐标系、误差定义相关的部分。完整的滚转控制、反步法控制律、自适应参数估计，不是在这两个文件里直接完成的。

---

## 2. 论文模型和代码目标的对应关系

论文把固定翼飞机在恒速、定高条件下简化为水平面单车模型：

```text
x_dot = V_chi * cos(chi)
y_dot = V_chi * sin(chi)
```

同时引入路径上的虚拟目标 `q(s)`，并在该点建立 Serret-Frenet 坐标系。

代码里对应的核心思想是：

1. 用弧长参数 `s` 描述虚拟目标在参考路径上的位置。
2. 对每个 `s`，都能求出：
   - 虚拟目标位置 `q(s)`
   - 该点路径切向角 `chi_f(s)`
   - 该点路径曲率 `kappa(s)`
3. 再把飞机当前位置相对 `q(s)` 的误差投影到 S-F 坐标系中，得到：
   - `es`
   - `ed`
   - `echi = chi - chi_f`

当前代码里最重要的一条更新律是：

```text
s_dot = ks * es + V_chi * cos(echi)
```

这正对应论文中的弧长推进律。

---

## 3. `flight_plan` 在这套实现中的含义

当前实现默认 `flight_plan` 每一行都描述一个航路点以及后续航段属性。工程里实际使用的列含义如下：

| 列号 | 含义 | 说明 |
| --- | --- | --- |
| 1 | 航段类型 | `0=IF`, `1=TF`, `2=RF` |
| 2 | 纬度 | 单位 `deg` |
| 3 | 经度 | 单位 `deg` |
| 4 | 高度 | 单位 `m`，当前这两个文件里只保存，不参与水平面更新 |
| 5 | 转弯方向 | RF 段使用，`-1` 表示左转，`+1` 表示右转 |
| 6 | 转弯半径 | RF 段使用，单位 `m` |
| 7 | 圆心纬度 | RF 段使用 |
| 8 | 圆心经度 | RF 段使用 |
| 9 | RNP 指标 | 当前这两个文件没有直接使用 |

所以，这两个文件本质上做的是：

> 根据 `flight_plan` 和飞机当前状态，持续维护“当前在第几段、虚拟目标到了哪里、该点路径几何是什么、飞机相对该点的误差是多少”。

---

## 4. `system_model_init.m` 是怎么建立初始系统模型的

`system_model_init.m` 负责创建 `sys_model` 结构体，并完成系统模型的第一次初始化。

它可以拆成 5 步来看。

### 4.1 写入常量和参数

文件开头先写入常量：

```matlab
sys_model.g = 9.81;
sys_model.R_earth = 6371000;
sys_model.deg2rad = pi / 180;
sys_model.rad2deg = 180 / pi;
```

这里的作用是：

- `g` 是重力加速度
- `deg2rad` 和 `rad2deg` 用于角度单位转换
- `R_earth` 当前没有直接参与本文件里的几何计算，但作为系统模型常量保留

然后写入 guidance law 会用到的参数：

```matlab
sys_model.k = 0.01;
sys_model.ks = 0.2;
sys_model.k_omega = 0.005;
sys_model.gamma = 4000;
sys_model.chi_inf = pi / 2;
```

这些值本身不是由当前文件推导出来的，而是作为控制设计参数预先放进结构体，方便后续调用。

---

### 4.2 保存飞行计划并计算航段数量

代码：

```matlab
sys_model.flight_plan = flight_plan;
sys_model.num_legs = size(flight_plan, 1) - 1;
```

含义：

- `flight_plan` 本身保存整个参考路径
- 如果有 `N` 个航路点，那么航段数就是 `N-1`

---

### 4.3 初始化虚拟目标状态

代码：

```matlab
sys_model.s = 0;
sys_model.leg_index = 1;
sys_model.q_lat = flight_plan(1, 2);
sys_model.q_lon = flight_plan(1, 3);
sys_model.q_alt = flight_plan(1, 4);
```

含义：

- 初始弧长参数 `s = 0`
- 初始航段编号 `leg_index = 1`
- 虚拟目标 `q(s)` 一开始就放在飞行计划第一个航路点

这里的逻辑非常直观：路径跟踪一开始，虚拟目标从参考路径起点出发。

---

### 4.4 计算初始路径几何量 `chi_f` 和 `kappa`

代码：

```matlab
[sys_model.chi_f, sys_model.kappa] = compute_path_geometry(sys_model, 1);
```

这里调用了本文件内部的局部函数 `compute_path_geometry`。

这个函数的任务是：

> 在指定航段的起点处，计算路径切向角 `chi_f` 和曲率 `kappa`。

#### 4.4.1 如果是 TF/IF 段

代码逻辑：

```matlab
leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
chi_f = wrapTo2Pi(leg_data(2) * deg2rad);
kappa = 0;
```

含义：

- 用 `func_GreatCircleInverse` 算起点到终点的大圆距离和初始方位角
- 把方位角转换成弧度，作为切向角 `chi_f`
- 曲率近似取 `0`

这说明当前实现把 TF/IF 段看成局部直线段。

严格来说，大圆航线的切向方向沿途会缓慢变化，因此这是一种工程近似，不是严格的全球大地几何模型。

#### 4.4.2 如果是 RF 段

代码逻辑：

```matlab
temp_start = func_GreatCircleInverse(center_lat, center_lon, start_lat, start_lon);
bearing_c2start = temp_start(2);
chi_f = wrapTo2Pi(bearing_c2start * deg2rad + turn_dir * pi / 2);
kappa = turn_dir / radius;
```

含义：

- 先算“圆心指向弧段起点”的方位角
- 圆弧切向方向等于径向方向再加一个 `90` 度的切向偏移
- 左转和右转方向由 `turn_dir` 决定
- 曲率不能只写成 `1 / radius`，而必须写成带方向的 `turn_dir / radius`

这一点非常关键。

因为路径切向角随弧长变化满足：

```text
dchi_f / ds = kappa
```

如果左转和右转都把 `kappa` 写成正值，那么 `chi_f` 的变化方向就会和真实圆弧不一致。

---

### 4.5 初始化误差状态和时间状态

代码：

```matlab
sys_model.es = 0;
sys_model.ed = 0;
sys_model.echi = 0;
sys_model.last_time = 0;
```

含义：

- 初始时刻认为误差缓存还没有历史值，所以统一置零
- `last_time` 用于下次更新时计算 `dt`

至此，`sys_model` 就具备了完整的初始系统模型状态。

---

## 5. `system_model_init.m` 输出的 `sys_model` 里有哪些字段

初始化后，`sys_model` 里主要保存以下内容：

| 字段 | 含义 |
| --- | --- |
| `g` | 重力加速度 |
| `deg2rad`, `rad2deg` | 角度转换系数 |
| `k`, `ks`, `k_omega`, `gamma`, `chi_inf` | guidance law 参数 |
| `flight_plan` | 飞行计划 |
| `num_legs` | 航段数 |
| `s` | 虚拟目标弧长参数 |
| `leg_index` | 当前航段编号 |
| `q_lat`, `q_lon`, `q_alt` | 虚拟目标位置 |
| `chi_f` | 当前虚拟目标处的路径切向角 |
| `kappa` | 当前虚拟目标处的路径曲率 |
| `es`, `ed`, `echi` | 当前缓存误差 |
| `last_time` | 上一次更新时间 |

---

## 6. `system_model_update.m` 是怎么做时序更新的

`system_model_update.m` 在每个仿真采样时刻被调用一次。

它的任务可以概括成一句话：

> 给定“飞机当前状态 + 当前时刻 + 上一时刻系统模型状态”，把虚拟目标、路径几何和误差状态一起推进到当前时刻。

---

## 7. `system_model_update.m` 的整体流程

这个文件的主流程可以分为 6 步。

### 7.1 读取飞机当前状态

代码：

```matlab
aircraft_lat = aircraft_state(1);
aircraft_lon = aircraft_state(2);
aircraft_chi = aircraft_state(3);
aircraft_V = aircraft_state(4);
```

然后做单位转换：

```matlab
chi = aircraft_chi * sys_model.deg2rad;
V_chi = aircraft_V;
```

也就是说，在当前实现里：

- 位置由纬度/经度给出
- 航迹角 `chi` 用弧度参与后续运算
- 速度直接用 `V_chi`

注意：

- `aircraft_state(5)` 预留给 `phi`
- 但当前这两个文件里没有直接使用它

---

### 7.2 计算时间步长 `dt`

代码：

```matlab
dt = current_time - sys_model.last_time;
if dt < 0
    error(...)
end
```

含义：

- `dt` 是当前采样和上一次更新之间的时间差
- 时间不能倒退，否则直接报错

---

### 7.3 构造整条路径的累计弧长表

主函数中调用：

```matlab
cumulative_dist = compute_cumulative_distance(sys_model);
```

这个局部函数会得到一个数组：

```text
cumulative_dist(i)
```

表示第 `i` 个航路点对应的累计路径长度。

#### 7.3.1 对 TF/IF 段的处理

代码逻辑：

```matlab
leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
leg_length = leg_data(1);
```

也就是直接把该航段长度取为起点到终点的大圆距离。

#### 7.3.2 对 RF 段的处理

代码逻辑：

```matlab
temp_start = func_GreatCircleInverse(center_lat, center_lon, start_lat, start_lon);
temp_end = func_GreatCircleInverse(center_lat, center_lon, end_lat, end_lon);
arc_angle_deg = func_CalculateArcAngle(temp_start(2), temp_end(2), turn_dir);
leg_length = radius * arc_angle_deg * sys_model.deg2rad;
```

即：

1. 算出圆心到起点和终点的方位角
2. 算出圆心角
3. 用弧长公式 `R * theta`

这样就把整条路径转换成了“按弧长索引”的形式。

这一步非常重要，因为后面只要知道 `s`，就能知道虚拟目标在整条路径的什么位置。

---

### 7.4 更新虚拟目标状态 `s`，并保证它和当前采样时刻一致

主函数调用：

```matlab
sys_model = update_virtual_target_state(sys_model, cumulative_dist, aircraft_lat, aircraft_lon, chi, V_chi, dt);
```

这是当前版本里最关键的一步。

#### 7.4.1 为什么不能简单写成显式欧拉

论文里：

```text
s_dot = ks * es + V_chi * cos(echi)
```

但这里有一个耦合关系：

- `s_dot` 依赖于 `es` 和 `echi`
- `es` 和 `echi` 又依赖于当前的虚拟目标位置 `q(s)`
- `q(s)` 又由 `s` 决定

所以 `s` 和误差不是彼此独立的。

如果简单写成：

```matlab
s_next = s_prev + s_dot_old * dt;
```

然后马上把 `q(s_next)` 返回给外部，就容易出现一个问题：

> 返回的几何量已经是下一步的，但误差还是按上一步算出来的。

在离散测试中，这种“错位”会很明显。

#### 7.4.2 当前实现如何解决这个问题

当前版本采用固定点迭代。

核心流程如下：

1. 先用旧的 `s` 对齐当前虚拟目标。
2. 如果 `dt = 0`，直接返回，不做推进。
3. 设置：

```matlab
s_prev = sys_model.s;
s_candidate = s_prev;
```

4. 在每轮迭代中：
   - 假设当前 `s = s_candidate`
   - 在这个假设下重新求虚拟目标和误差
   - 再用

```matlab
s_next = s_prev + dt * (ks * es_iter + V_chi * cos(echi_iter));
```

   更新一次 `s`
5. 如果 `s_next` 和 `s_candidate` 足够接近，就认为收敛。
6. 最终把收敛后的 `s` 写回 `sys_model`。

这样做的结果是：

> 返回给外部的 `q(s)`、`chi_f`、`kappa`、`es`、`ed`、`echi` 尽量属于同一个采样时刻。

这就是当前版本里“系统模型状态自洽”的关键。

---

### 7.5 `sync_virtual_target` 是怎样把 `s` 转成几何状态的

`update_virtual_target_state` 内部会调用：

```matlab
sys_model = sync_virtual_target(sys_model, cumulative_dist);
```

这个函数的作用是：

> 根据当前 `s`，同步出当前航段编号、虚拟目标位置、路径切向角和曲率。

它具体做了几件事。

#### 7.5.1 限制 `s` 的范围

```matlab
sys_model.s = min(max(sys_model.s, 0), cumulative_dist(end));
```

即：

- 不允许虚拟目标跑到路径起点之前
- 不允许虚拟目标跑到路径终点之后

#### 7.5.2 判断 `s` 落在哪一段

代码分两部分：

```matlab
while sys_model.leg_index > 1 && sys_model.s < cumulative_dist(sys_model.leg_index)
    sys_model.leg_index = sys_model.leg_index - 1;
end
```

和

```matlab
while sys_model.leg_index < sys_model.num_legs && ...
      sys_model.s >= cumulative_dist(sys_model.leg_index + 1)
    sys_model.leg_index = sys_model.leg_index + 1;
end
```

含义：

- 如果当前 `s` 比当前段起点还小，就往前退一段
- 如果当前 `s` 已经越过当前段终点，就往后推一段

因此，`leg_index` 始终和 `s` 保持一致。

#### 7.5.3 计算当前段内的局部弧长

```matlab
local_s = sys_model.s - cumulative_dist(sys_model.leg_index);
```

`local_s` 表示：

> 虚拟目标在当前航段内部已经走了多少距离。

#### 7.5.4 由 `local_s` 求出当前几何状态

最后调用：

```matlab
[sys_model.q_lat, sys_model.q_lon, sys_model.chi_f, sys_model.kappa] = ...
    compute_virtual_target(sys_model, sys_model.leg_index, local_s);
```

这样，`s` 就被转成了真正可用于误差计算的几何状态。

---

## 8. `compute_virtual_target` 是如何从 `s` 求出 `q(s)` 的

这是 `system_model_update.m` 里最核心的几何函数。

它的任务是：

> 已知当前航段 `leg_index` 和当前段内弧长 `local_s`，求出虚拟目标位置 `q_lat/q_lon`，以及该点路径切向角 `chi_f` 和曲率 `kappa`。

### 8.1 对 RF 段的实现

RF 段时，代码步骤如下。

#### 第一步：读取圆弧参数

```matlab
center_lat = flight_plan(leg_index, 7);
center_lon = flight_plan(leg_index, 8);
radius = flight_plan(leg_index, 6);
turn_dir = flight_plan(leg_index, 5);
```

#### 第二步：求整段圆弧总长

```matlab
temp_start = func_GreatCircleInverse(center_lat, center_lon, start_lat, start_lon);
temp_end = func_GreatCircleInverse(center_lat, center_lon, end_lat, end_lon);
arc_angle_deg = func_CalculateArcAngle(temp_start(2), temp_end(2), turn_dir);
arc_length = radius * arc_angle_deg * deg2rad;
```

#### 第三步：限制 `local_s`

```matlab
local_s = min(max(local_s, 0), arc_length);
```

#### 第四步：根据弧长算圆心到当前点的方位角

```matlab
bearing_c2s = temp_start(2);
delta_angle_deg = turn_dir * (local_s / radius) * rad2deg;
bearing_c2q = wrapTo360Deg(bearing_c2s + delta_angle_deg);
```

这里本质上是在做：

```text
theta = s / R
```

即：

- 已知弧长 `s`
- 已知半径 `R`
- 先求当前点相对起点转过了多少角度

#### 第五步：根据圆心、方位角、半径求虚拟目标位置

```matlab
q = func_GreatCincleForward(center_lat, center_lon, bearing_c2q, radius);
q_lat = q(1);
q_lon = q(2);
```

#### 第六步：求切向角和曲率

```matlab
chi_f = wrapTo2Pi(bearing_c2q * deg2rad + turn_dir * pi / 2);
kappa = turn_dir / radius;
```

所以 RF 段里：

- 位置由“圆心 + 半径 + 当前转角”确定
- 切向角由径向角加上切向修正得到
- 曲率带方向

---

### 8.2 对 TF/IF 段的实现

TF/IF 段时代码更简单：

```matlab
leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
leg_bearing = leg_data(2);
leg_length = leg_data(1);
local_s = min(max(local_s, 0), leg_length);

q = func_GreatCincleForward(start_lat, start_lon, leg_bearing, local_s);
q_lat = q(1);
q_lon = q(2);
chi_f = wrapTo2Pi(leg_bearing * deg2rad);
kappa = 0;
```

含义：

- 沿航段初始方位角前推 `local_s`
- 得到虚拟目标位置
- 切向角近似固定为该航段初始方位角
- 曲率近似为 `0`

这依然是“局部直线近似”。

---

## 9. `compute_tracking_errors` 是如何计算 `es`、`ed`、`echi` 的

这个函数负责把飞机当前位置相对虚拟目标的偏差投影到 S-F 坐标系里。

步骤如下。

### 9.1 先算虚拟目标到飞机的几何关系

```matlab
temp = func_GreatCircleInverse(sys_model.q_lat, sys_model.q_lon, aircraft_lat, aircraft_lon);
dist_q2aircraft = temp(1);
bearing_q2aircraft_rad = temp(2) * sys_model.deg2rad;
```

得到：

- `dist_q2aircraft`：虚拟目标到飞机的距离
- `bearing_q2aircraft_rad`：从虚拟目标指向飞机的方位角

### 9.2 计算航迹角误差

```matlab
echi = wrapToPi(chi - chi_f);
```

也就是：

```text
echi = chi - chi_f
```

并包装到 `[-pi, pi]`。

### 9.3 计算相对切向方向的夹角

```matlab
delta_bearing = wrapToPi(bearing_q2aircraft_rad - chi_f);
```

这个量表示：

> “虚拟目标指向飞机的连线”相对于“路径切向方向”的夹角。

### 9.4 做切向/法向投影

```matlab
ed = dist_q2aircraft * sin(delta_bearing);
es = dist_q2aircraft * cos(delta_bearing);
```

含义：

- `es` 是沿路径切向的投影
- `ed` 是垂直路径切向的投影

当前代码的符号约定是：

- `es > 0`：飞机在虚拟目标前方
- `es < 0`：飞机在虚拟目标后方
- `ed < 0`：飞机在路径左侧
- `ed > 0`：飞机在路径右侧

这个符号约定一定要和后续 guidance law 保持一致。

---

## 10. `system_model_update.m` 最后输出了什么

更新结束后，函数会把最新状态写回 `sys_model`：

```matlab
sys_model.es = es;
sys_model.ed = ed;
sys_model.echi = echi;
sys_model.last_time = current_time;
```

然后把当前时刻的结果打包成 `errors` 结构体输出：

| 字段 | 含义 |
| --- | --- |
| `es` | 沿迹误差 |
| `ed` | 横迹误差 |
| `echi` | 航迹角误差 |
| `s` | 当前虚拟目标弧长参数 |
| `leg_index` | 当前航段编号 |
| `chi_f` | 当前虚拟目标切向角 |
| `kappa` | 当前虚拟目标曲率 |
| `s_dot` | 当前误差对应的弧长变化率 |
| `s_dot_used` | 当前版本中与 `s_dot` 相同，保留字段便于兼容调用端 |
| `q_lat`, `q_lon` | 当前虚拟目标位置 |

因此，外部测试文件和 guidance wrapper 可以直接消费 `errors`，而不用再从 `sys_model` 里逐个取字段。

---

## 11. 从调用顺序看，这两个文件是怎样配合工作的

把它们连起来看，完整流程如下：

1. 外部先调用：

```matlab
sys_model = system_model_init(flight_plan);
```

2. 得到完整初始化后的系统模型状态。

3. 每个采样时刻，把飞机当前状态送入：

```matlab
[sys_model, errors] = system_model_update(sys_model, aircraft_state, current_time);
```

4. `system_model_update` 做：
   - 更新时间差
   - 构建累计弧长表
   - 更新弧长参数 `s`
   - 同步虚拟目标 `q(s)`
   - 求切向角 `chi_f`
   - 求曲率 `kappa`
   - 求误差 `es`、`ed`、`echi`

5. 外部拿到 `errors` 后，就能继续做控制律计算或测试可视化。

所以可以把这两个文件理解成：

- `system_model_init.m`：负责“搭好系统模型状态框架”
- `system_model_update.m`：负责“把系统模型状态推进到当前时刻”

---

## 12. 当前实现的假设和边界

这部分很重要，因为它决定了“代码实现到什么程度了”。

### 12.1 当前实现只覆盖了水平几何层

这两个文件没有直接实现：

- 滚转动态方程
- 反步法控制器
- 参数自适应律

它们只负责给这些上层控制环节提供几何和误差状态。

### 12.2 `phi` 目前没有参与更新

虽然 `aircraft_state` 预留了 `phi`，但当前文件中还未直接使用。

### 12.3 TF/IF 段是工程近似

当前把 TF/IF 段近似看成零曲率段：

```matlab
kappa = 0;
chi_f = leg_bearing;
```

对于普通仿真和局部路径段，这通常够用；但如果需要非常严格的大圆几何一致性，后续还可以继续增强。

### 12.4 RF 段的曲率方向必须带符号

这一点已经在当前版本里修正成：

```matlab
kappa = turn_dir / radius;
```

如果不带方向，左转和右转的几何关系会错。

### 12.5 当前版本强调“采样时刻状态自洽”

更新 `s` 的时候采用固定点迭代，不是为了炫技，而是为了避免：

- `q(s)` 属于下一时刻
- `es/ed/echi` 却还是上一时刻

这种错位在测试图里会非常明显，因此当前版本特意做了“当前时刻几何量和误差量同步”。

---

## 13. 一句话总结

一句话总结这两个文件的职责：

> `system_model_init.m` 负责把路径和系统模型初值搭起来，`system_model_update.m` 负责在每个采样时刻把虚拟目标、路径几何和误差状态推进到当前时刻并保持一致。

两者合起来，就构成了论文中水平面路径跟踪系统模型在当前工程里的代码实现。

