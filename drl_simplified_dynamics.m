function [A, B, F, C, G] = drl_simplified_dynamics()
% drl_simplified_dynamics - 横航向小扰动线性模型
% A、B 通过 Simulink 配平加线性化获得
% C = eye(4)，表示状态直接输出
% F 暂时保留为与旧接口兼容的近似定义 F = A(:,1)

cd('d:/桌面/615_items/y8模型/y8ModelAllControl');

% 与 Start.m 保持一致的工况
mass = 54000;
Vt = 80;
alt = 1284.73;

% 将工况变量显式送入 base workspace，
% 以便 AircraftTrim 内部调用 sim(...) 时能正确读取 mass、X、Y 等变量
assignin('base', 'mass', mass);
assignin('base', 'Vt', Vt);
assignin('base', 'alt', alt);

% 在 base workspace 中执行 AircraftTrim。
% AircraftTrim 是脚本，其中会继续调用 Inertia_Calculate_54 等模型，
% 这些模型块参数默认从 base workspace 取值
evalin('base', 'AircraftTrim;');
op = evalin('base', 'op');

% 线性化
modelname = 'Aero_Dynamics';
i = 1;
io(i) = linio([modelname, '/delta_a'], 1, 'in'); i = i + 1;
io(i) = linio([modelname, '/delta_e'], 1, 'in'); i = i + 1;
io(i) = linio([modelname, '/delta_r'], 1, 'in'); i = i + 1;
io(i) = linio([modelname, '/delta_p'], 1, 'in'); i = i + 1;
for k = 1:12
    io(i) = linio([modelname, '/U1'], k, 'out');
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

% C = 单位阵，G = 零向量
C = eye(4);
G = zeros(4, 1);

% 为保持旧接口兼容，暂时继续保留近似定义
F = A(:, 1);
end
