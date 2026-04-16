# drl_simplified_dynamics 重写为 Simulink 线性化源

**Goal:** 将 A/B/C 矩阵的来源从手选气动导数公式切换为 Simulink 配平+线性化结果，函数签名不变。

**Architecture:** `drl_simplified_dynamics()` 内部做 `lateralFunction.m` 做的事情，返回 A,B,F,C,G。

---

## Task 1: 重写 drl_simplified_dynamics.m

**文件:** `drl_simplified_dynamics.m`

旧版（行 11-80）全部删除，替换为 `lateralFunction.m` 的线性化链：

```matlab
function [A, B, F, C, G] = drl_simplified_dynamics()
% drl_simplified_dynamics - 横航向小扰动线性模型
%
% A, B 通过 Simulink 配平+线性化获取
% C = eye(4)，状态直接输出
% F 暂时保留 F = A(:,1) 近似

cd('d:/桌面/615_items/y8模型/y8ModelAllControl');

% 和 Start.m 保持一致的工况
mass = 54000;
Vt = 80;
alt = 1284.73;

% 配平
AircraftTrim;

% 线性化
modelname = 'Aero_Dynamics';
i = 1;
io(i) = linio([modelname,'/delta_a'],1,'in'); i = i + 1;
io(i) = linio([modelname,'/delta_e'],1,'in'); i = i + 1;
io(i) = linio([modelname,'/delta_r'],1,'in'); i = i + 1;
io(i) = linio([modelname,'/delta_p'],1,'in'); i = i + 1;
for k = 1:12
    io(i) = linio([modelname,'/U1'],k,'out');
    i = i + 1;
end

sys = linearize(modelname, op, io);

A_full = sys.A;
B_full = sys.B;

% 横侧向子系统：状态 [Beta, p, r, phi]，输入 [delta_a, delta_r]
A_lat = A_full([3 4 6 7], [3 4 6 7]);
B_lat = B_full([3 4 6 7], [1 3]);

A = A_lat;
B = B_lat;

% C = I, G = 0
C = eye(4);
G = zeros(4, 1);

% F 暂时保留近似定义
F = A(:, 1);
end
```

---

## Task 2: 重新生成 .mat 文件

在 MATLAB 中执行：
```matlab
cd('d:/桌面/615_items/y8模型/y8ModelAllControl');
[A, B, F] = drl_simplified_dynamics();
save('drl_dynamics_matrices.mat', 'A', 'B', 'F', '-v7.3');
```

---

## Task 3: 用 lateralFunction.m 交叉验证

运行 `lateralFunction.m`，对比：
- `lateralFunction` 打印的 `A_lat` vs `drl_simplified_dynamics()` 返回的 `A` → 完全一致
- `B_lat` vs `B` → 完全一致

---

## Task 4: 验证调用方兼容性

| 调用方 | 取用内容 | 兼容性 |
|--------|---------|--------|
| `drl_rollout_env.m` | `[A,B,F,~,~]` | 兼容（4×4, 4×2, 4×1 不变） |
| `drl_test_dynamics.m` | `[A,B,F,C,G]` | C 变为 4×4，需检查测试断言 |
| Python `.py` | 仅从 .mat 加载 A,B,F | 不受影响 |
