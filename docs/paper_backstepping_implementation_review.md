# 论文《L1-BackStepping》反步法在当前飞机模型中的具体实现审查

本文档以 [L1-BackStepping.pdf](D:/桌面/L1L1论文/L1-BackStepping.pdf) 为基准，说明论文中的反步控制思想在当前飞机模型里的对应实现、实际接线状态，以及与论文原式之间的差异，供审核使用。

## 1. 审查结论

先给结论，便于你快速把握重点：

1. 当前 `AFCSModel` 里，论文中的**完整反步法主链并没有真正接入飞机主控制回路**。
2. 当前真正在线运行的横向主链，更接近于：
   - 论文的运动学控制层 `Eq.(5) + Eq.(6a) + Eq.(6b)`
   - 再加上 `Eq.(7)` 的静态换算 `phi_d = atan(V * omega_d / g)`
3. 模型中确实存在两个“反步”子系统：
   - `backSteppingControl`
   - `backSteppingControlTanh`
   但它们目前在顶层**只有输入，没有输出接到 `RNPAR_phi_guidance`**，因此不是当前飞机真正执行的横向指令来源。
4. 论文中反步法最关键的两部分：
   - 用滚转动态补偿的辅助控制 `Eq.(11)`
   - 闭环滚转时间常数的自适应估计 `Eq.(17) ~ Eq.(24)`
   在当前主控制链里**没有完整实现为在线工作的模块**。

所以，如果问题是“论文中的反步法在当前飞机模型里有没有被真正完整实现并驱动飞机”，结论是：

**没有完整落成。当前真正驱动飞机的是‘运动学层 + 静态滚转换算’，不是论文的完整自适应反步法。**

---

## 2. 论文中的控制结构是什么

结合论文第 2 节，控制结构可以分成四层：

### 2.1 系统模型

论文 `Eq.(1) ~ Eq.(4)` 给出了固定翼飞机在恒高、恒速假设下的简化模型：

- 平面运动学：`x_dot`, `y_dot`, `chi_dot = omega`
- 协调转弯关系：`omega = g/V * tan(phi)`
- 滚转一阶动态：`phi_dot = (phi_c - phi) / lambda_phi`
- 在 Serret-Frenet 坐标系下的误差动态：`es`, `ed`, `chi_e`

### 2.2 运动学控制层

论文 `Eq.(5)` 定义逼近角 `delta(ed)`，`Eq.(6a)` 给出期望航迹角速度 `omega_d`，`Eq.(6b)` 给出虚拟目标推进速度 `s_dot`。

这一层的作用是：

- 先从路径误差 `es / ed / chi_e`
- 生成一个理想的航迹角速度命令 `omega_d`
- 同时推进虚拟目标 `q(s)`

### 2.3 反步控制层

论文 `Eq.(7)` 先给出最直接的静态换算：

`phi_d = atan(V * omega_d / g)`

但论文认为这还不够，因为真实飞机滚转有一阶动态 `lambda_phi`，所以又引入：

- 辅助控制 `nu = phi_dot`
- 航迹角速度误差 `omega_e = omega - omega_d`
- 通过反步法设计 `Eq.(11)`，补偿滚转动态

这一步才是论文标题里“Backstepping”的核心。

### 2.4 自适应与有界控制

论文进一步做了两件事：

1. 用 `Eq.(17) ~ Eq.(24)` 在线估计 `lambda_phi`
2. 用有界输入策略限制滚转角，避免直接饱和破坏自适应过程

因此，论文中的“完整反步法”不是只有一个 `atan(V*omega/g)`，而是：

`运动学层 -> 反步滚转补偿 -> 参数自适应 -> 有界控制`

---

## 3. 当前模型里有哪些对应模块

当前工作区里，与这篇论文最相关的文件和模块是：

- [AFCSModel.mdl](D:/桌面/615_items/y8模型/y8ModelAllControl/AFCSModel.mdl)
- [func_ApproachGuidance_RNPAR.m](D:/桌面/615_items/y8模型/y8ModelAllControl/func_ApproachGuidance_RNPAR.m)
- [system_model_init.m](D:/桌面/615_items/y8模型/y8ModelAllControl/system_model_init.m)
- [system_model_update.m](D:/桌面/615_items/y8模型/y8ModelAllControl/system_model_update.m)
- [func_KinematicControlLaw.m](D:/桌面/615_items/y8模型/y8ModelAllControl/func_KinematicControlLaw.m)
- [func_SystemModelMonitorBlock.m](D:/桌面/615_items/y8模型/y8ModelAllControl/func_SystemModelMonitorBlock.m)

在 `AFCSModel` 的 `RNP AR Approach Guidance` 子系统中，可以看到五条横向引导分支：

- `Lateral_controll`
- `backSteppingControl`
- `backSteppingControlTanh`
- `ndiControl`
- `L1GuidanceVersion1`

但从顶层连线来看，目前真正接到 `RNPAR_phi_guidance` 的，不是上面这些子系统的输出，而是 `func_SystemModelMonitorBlock -> omega_d -> atan(V*omega_d/g)` 这条链。

---

## 4. 当前真正在线运行的横向主链

这是当前模型里真正驱动飞机横向滚转指令的链路。

## 4.1 路径误差和虚拟目标

[func_SystemModelMonitorBlock.m](D:/桌面/615_items/y8模型/y8ModelAllControl/func_SystemModelMonitorBlock.m) 内部调用：

- [system_model_init.m](D:/桌面/615_items/y8模型/y8ModelAllControl/system_model_init.m)
- [system_model_update.m](D:/桌面/615_items/y8模型/y8ModelAllControl/system_model_update.m)

它完成的事情是：

- 根据飞行计划建立参考路径
- 推进虚拟目标 `q(s)`
- 计算 `es / ed / echi / chi_f / kappa`

这部分对应论文的：

- `Eq.(4)` 误差动力学建模背景
- `q(s)`、`chi_f(s)`、`kappa(s)` 的路径几何定义

## 4.2 论文运动学控制律

[func_KinematicControlLaw.m](D:/桌面/615_items/y8模型/y8ModelAllControl/func_KinematicControlLaw.m) 明确实现了论文的运动学层：

- `delta(ed)` 对应论文 `Eq.(5)`
- `s_dot` 对应论文 `Eq.(6b)`
- `omega_d` 对应论文 `Eq.(6a)`

也就是说，当前模型里**论文的运动学引导层是有代码实现的，而且已经在线使用**。

## 4.3 静态滚转换算

`AFCSModel` 顶层当前把 `func_SystemModelMonitorBlock` 的第 11 个输出 `omega_d` 取出来后，走的是：

1. 乘以地速 `Vd`
2. 除以 `g`
3. 过 `atan`
4. 送到 `RNPAR_phi_guidance`

这正对应论文 `Eq.(7)`：

`phi_d = atan(V * omega_d / g)`

因此，**当前真正在线执行的是论文的“运动学层 + Eq.(7) 静态滚转换算”**。

## 4.4 这条主链缺少什么

它缺少的是论文反步法最关键的两部分：

- 没有显式实现 `omega_e = omega - omega_d` 的反步辅助控制主律 `Eq.(11)`
- 没有在线估计 `lambda_phi` 的自适应律 `Eq.(24)`

所以这条当前在线主链**不是论文完整反步法**。

---

## 5. 模型中留存的两个“反步”子系统

虽然当前主链不是论文完整反步法，但模型中确实有两个反步风格的横向模块：

- `backSteppingControl`
- `backSteppingControlTanh`

这两个模块都位于 [AFCSModel.mdl](D:/桌面/615_items/y8模型/y8ModelAllControl/AFCSModel.mdl) 的 `RNP AR Approach Guidance` 子系统内。

## 5.1 这两个模块的输入是什么

从顶层连线可以看出，这两个子系统的 4 个输入完全一致，都是从原始 RNP 引导输出中取的：

1. `legType`
2. `Chi_g`
3. `YZ`
4. `phi_g_in`

其中：

- `legType` 来自 `RNPAR_CurLegInfo(4)`
  - `0` 表示 TF
  - `1` 表示 RF
- `Chi_g` 来自 `RNPAR_GuidePara(2)`
  - 即参考航迹角
- `YZ` 来自 `RNPAR_GuidePara(1)`
  - 即横向偏差
- `phi_g_in` 来自 `RNPAR_GuidePara(3)`
  - TF 航段通常为 `0`
  - RF 航段为几何前馈滚转角

这些信号都来自 [func_ApproachGuidance_RNPAR.m](D:/桌面/615_items/y8模型/y8ModelAllControl/func_ApproachGuidance_RNPAR.m)。

这个函数输出的 `Guidance_Para` 顺序是：

- `Guidance_Para(1) = Para_XTK`
- `Guidance_Para(2) = Para_chi`
- `Guidance_Para(3) = Para_phi`
- `Guidance_Para(4) = Para_H`
- `Guidance_Para(5) = Para_VS`

因此，两个反步子系统实际上是建立在**旧的 RNP AR 引导量**之上的，不是建立在 `system_model_update` 算出来的 `es / ed / echi / q(s)` 之上的。

---

## 6. `backSteppingControl` 子系统到底实现了什么

这个模块对应 `system_5343` 和其内部 `caculatePhi`。

## 6.1 输入量的物理意义

模块内部会从 `Plane_Data` 中再取：

- 当前航迹角 `Chi`
- 当前地速 `Vd`

并构造：

- `deltaChi = Chi - Chi_g`

这里的 `deltaChi` 本质上就是“当前航迹角相对参考航迹角的误差”。

## 6.2 模块内部的计算结构

按 Simulink 连线化简后，这个模块的核心形式可以写成：

```text
u_y   = ((k1*k2)/V + eps*V) * YZ
u_chi = (k1 + k2) * sin(deltaChi)
u_fb  = u_y - u_chi
phi_bs = atan( V/(g*cos(deltaChi)) * u_fb )
```

其中当前模型参数为：

- `k1 = 0.02`
- `k2 = 0.3`
- `eps = 5e-6`

如果当前航段是 RF，则它还会把几何前馈滚转角 `phi_g_in` 加进去：

```text
phi_cmd_RF = atan( tan(phi_g_in) + V/(g*cos(deltaChi)) * u_fb )
```

最后再经过：

- `phiLimit = sat(phi_cmd, ±30 deg)`

## 6.3 这个模块和论文反步法的关系

这个模块**有反步风格**，但它并不是论文 `Eq.(11) + Eq.(17) + Eq.(24)` 的直接落地版本。

原因是：

1. 它没有显式构造论文里的 `omega_e = omega - omega_d`
2. 它没有显式使用 `omega_d_dot`
3. 它没有 `lambda_phi` 或 `lambda_phi_hat`
4. 它没有参数更新律
5. 它更像是“横向偏差 + 航迹角误差 -> 滚转角”的静态非线性综合器

所以它与论文的关系应理解为：

**思路相近，但不是论文原式的完整实现。**

---

## 7. `backSteppingControlTanh` 子系统到底实现了什么

这个模块对应 `system_5463`、`caculateZ1F1`、`caculateZ2` 和 `Subsystem(5563)`。

它比前一个模块多了一层有界化处理。

## 7.1 第一步：构造有界横向误差 `z1`

内部首先计算：

```text
z1 = tanh(Kp * YZ)
```

当前模型中：

- `Kp = 0.002`

这一步的作用是：

- 当 `YZ` 很大时，`z1` 不再无限增大
- 让误差进入有界区间

这和论文“bounded bank angle / bounded control”部分在设计思想上是一致的，但形式上不是论文原文的完整照搬。

## 7.2 第二步：构造 `f1`

模块 `caculateZ1F1` 又计算：

```text
f1 = (1 - z1^2) * Kp * V
```

这是 `tanh(Kp*YZ)` 对 `YZ` 的导数，再乘以速度项得到的中间量。

## 7.3 第三步：构造第二层误差 `z2`

模块 `caculateZ2` 计算：

```text
z2 = sin(deltaChi) - 0.25 * z1
```

这里可以理解为把“航迹角误差”和“横向误差的有界表示”耦合起来。

## 7.4 第四步：生成滚转命令

内部最终形成的反馈项可以整理为：

```text
u_fb = -0.25*z2 + z1*f1 - 0.25*f1*sin(deltaChi)
```

再经静态换算得到：

```text
phi_bs_tanh = atan( V/(g*cos(deltaChi)) * u_fb )
```

RF 航段时同样加入几何前馈：

```text
phi_cmd_RF = atan( tan(phi_g_in) + V/(g*cos(deltaChi)) * u_fb )
```

最后也会经过：

- `phiLimit = sat(phi_cmd, ±30 deg)`

## 7.5 这个模块和论文反步法的关系

这个模块比 `backSteppingControl` 更强调“有界输入”和“误差压缩”，与论文题目里的 “Bounded Bank Angle” 更接近。

但它依然不是论文 `Eq.(17) ~ Eq.(24)` 的完整实现，因为：

- 没有 `lambda_phi_hat`
- 没有参数更新律
- 没有显式的 `omega_e` 状态反馈项
- 没有 `omega_d_dot` 的滤波器实现

因此，它更准确的定位应是：

**一个带 `tanh` 有界化的反步风格滚转命令模块，而不是论文自适应反步法的完整工程复现。**

---

## 8. 这两个反步模块当前有没有真正驱动飞机

结论非常明确：

**没有。**

我检查了 `AFCSModel` 顶层连线，结果是：

- 能找到很多 `5343#in:*` 和 `5463#in:*`，说明这两个模块有输入
- 但找不到 `5343#out:1` 或 `5463#out:1` 接到 `RNPAR_phi_guidance`
- 当前 `RNPAR_phi_guidance` 实际上是由 `func_SystemModelMonitorBlock` 输出的 `omega_d` 经 `atan(V*omega_d/g)` 后送出的

这意味着：

- `backSteppingControl` 存在
- `backSteppingControlTanh` 存在
- 但当前版本里它们都不是飞机在线使用的横向指令源

这也是为什么从“当前实际飞行主链”的角度看，你的模型还不能说已经完整实现了论文中的反步法。

---

## 9. 当前滚转指令最终是怎样进入飞机模型的

当前实际进入飞机的横向指令路径是：

1. `RNP AR Approach Guidance` 生成 `RNPAR_phi_guidance`
2. 根模型通过 `From4` 读取 `RNPAR_phi_guidance`
3. 送入 `Roll_modify`
4. `Roll_modify` 结合当前 `phi / p / r` 做低层滚转控制
5. 输出 `delta_a` 和 `delta_r`
6. 进入 `Plane&Actuator`
7. 经执行机构和飞机气动模型作用到飞机姿态

这里的 [AFCSModel.mdl](D:/桌面/615_items/y8模型/y8ModelAllControl/AFCSModel.mdl) 中 `Roll_modify` 本质上是一个独立的低层滚转环，它会：

- 对滚转指令做 `±30 deg` 饱和
- 用当前滚转角 `phi` 做误差反馈
- 用滚转角速度 `p` 和偏航角速度 `r` 做附加反馈
- 输出副翼/方向舵相关控制量

因此，论文反步法如果以后要真正落地到飞机上，应该是：

- 先在高层正确生成 `phi_c`
- 再把 `phi_c` 接到 `Roll_modify`

而不是只把反步模块放在 `RNP AR Approach Guidance` 里但不接出。

---

## 10. 按论文标准审查，当前模型已经实现了什么、缺了什么

## 10.1 已实现并在线工作的部分

### 已实现 1：路径误差与虚拟目标

已由：

- [system_model_init.m](D:/桌面/615_items/y8模型/y8ModelAllControl/system_model_init.m)
- [system_model_update.m](D:/桌面/615_items/y8模型/y8ModelAllControl/system_model_update.m)

实现 `q(s)`、`es`、`ed`、`echi`、`chi_f`、`kappa` 的计算。

### 已实现 2：论文运动学控制律

已由：

- [func_KinematicControlLaw.m](D:/桌面/615_items/y8模型/y8ModelAllControl/func_KinematicControlLaw.m)

实现 `Eq.(5)`、`Eq.(6a)`、`Eq.(6b)`。

### 已实现 3：`Eq.(7)` 静态滚转换算

已在 [AFCSModel.mdl](D:/桌面/615_items/y8模型/y8ModelAllControl/AFCSModel.mdl) 顶层工作：

```text
phi_d = atan(V * omega_d / g)
```

## 10.2 只做了局部近似、但没有完整按论文实现的部分

### 部分实现 1：反步风格滚转命令

`backSteppingControl` 和 `backSteppingControlTanh` 都能算出一个反步风格 `phi` 命令，但它们：

- 没有直接对应论文完整公式
- 当前也没有接入主控制链

### 部分实现 2：有界控制思想

两个反步模块内部都有限幅 `±30 deg`，其中 `Tanh` 版本还做了有界误差压缩。

但这不等同于论文 `Eq.(28) ~ Eq.(30)` 那种“通过限制期望转弯率/运动学层命令来保持自适应一致性”的完整有界控制策略。

## 10.3 当前没有实现的部分

### 未实现 1：论文 `Eq.(11)` 的完整在线反步辅助控制

当前在线主链没有看到完整的：

- `omega_e`
- `omega_d_dot`
- `nu`
- 通过滚转一阶动态补偿得到 `phi_c`

这组完整结构。

### 未实现 2：论文 `Eq.(17) ~ Eq.(24)` 的参数自适应

当前工作区没有看到与以下量对应的在线实现：

- `lambda_phi_hat`
- `lambda_phi_hat_dot`
- 参数误差状态
- 估计更新律

### 未实现 3：`omega_d_dot` 的滤波微分器

论文在实现层建议对 `omega_d_dot` 使用滤波微分，而当前主链里没有发现这部分。

---

## 11. 最终判断

如果用一句话概括当前模型与论文的关系：

**当前模型已经实现了论文的路径误差建模和运动学层，并且实现了一个在线工作的 `Eq.(7)` 静态滚转换算；模型里也存在两个反步风格的滚转命令模块，但它们既不是论文完整公式的工程复现，也没有真正接入当前飞机主回路，因此不能把当前 AFCSModel 视为“已经完整实现了论文中的自适应反步法”。**

更具体地说：

- 从“算法思想”上看：你已经走到了论文的一大半
- 从“当前在线工作主链”上看：还停留在“运动学层 + 静态滚转换算”
- 从“完整论文复现”上看：还缺真正接入的反步辅助控制、自适应时间常数估计，以及论文式有界控制主链

---

## 12. 如果你后续要继续按论文补全，最直接的补全路线

建议按下面顺序做，而不是同时大改：

1. 先确定最终要启用哪一条高层横向链
   - 如果按论文走，应让论文反步链成为唯一的 `RNPAR_phi_guidance` 来源

2. 把 `omega_d`、`omega_e`、`omega_d_dot`、`nu`、`phi_c` 的状态链单独拉清楚
   - 不要再和旧的 `RNPAR_GuidePara` 静态支路混用

3. 明确 `lambda_phi_hat` 的状态与更新位置
   - 需要一个真正在线更新的估计状态，而不是固定增益

4. 再决定有界控制放在哪一层
   - 是像论文那样限制运动学层命令
   - 还是像你当前子系统那样直接限制 `phi`

如果你愿意，我下一步可以继续直接给你写第二份文档：

**《按论文 Eq.(11)/(17)/(24) 应该如何在你当前 AFCSModel 中重构接线》**

那份会更偏“改模型实施清单”。  
