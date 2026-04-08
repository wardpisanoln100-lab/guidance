% test_system_model.m - 测试System Model (直线段)
clc; clear; close all;
warning off;

fprintf('===== 测试 System Model (直线段) =====\n\n');

% ===== 1. 加载飞行计划 =====
Manuscript_RNPAR_FlightPlan;
flight_plan = RNPAR_FlightPlan;

% ===== 2. 初始化System Model =====
fprintf('--- 初始化System Model ---\n');
sys_model = system_model_init(flight_plan);

% ===== 3. 模拟飞机沿第一段直线飞行 =====
fprintf('\n--- 仿真测试 ---\n');

% 起点和终点
start_lat = flight_plan(1, 2);
start_lon = flight_plan(1, 3);
end_lat = flight_plan(2, 2);
end_lon = flight_plan(2, 3);

% 计算直线参数
[leg_data] = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
bearing = leg_data(2);
leg_length = leg_data(1);

fprintf('第一段直线: 长度=%.1f m, 方位角=%.2f deg\n', leg_length, bearing);

% 存储轨迹数据
trajectory_lat = [];
trajectory_lon = [];
target_lat = [];
target_lon = [];
es_hist = [];
ed_hist = [];
echi_hist = [];
s_hist = [];
s_dot_hist = [];
leg_hist = [];

% 仿真时间
dt = 0.01;  % 时间步长
total_time = 12;

fprintf('\n时间(s) | es(m)   | ed(m)   | echi(rad) | s(m)    | 航段\n');
fprintf('-------------------------------------------------------------\n');

for t = 0:dt:total_time
    % 飞机位置：沿直线前进
    progress = min(t * 80, leg_length);  % 速度80m/s
    aircraft_pos = func_GreatCincleForward(start_lat, start_lon, bearing, progress);

    aircraft_lat = aircraft_pos(1);
    aircraft_lon = aircraft_pos(2);
    aircraft_chi = bearing;
    aircraft_V = 80;

    aircraft_state = [aircraft_lat, aircraft_lon, aircraft_chi, aircraft_V, 0];
    [sys_model, errors] = system_model_update(sys_model, aircraft_state, t);

    % 记录轨迹
    trajectory_lat = [trajectory_lat, aircraft_lat];
    trajectory_lon = [trajectory_lon, aircraft_lon];
    target_lat = [target_lat, errors.q_lat];
    target_lon = [target_lon, errors.q_lon];
    es_hist = [es_hist, errors.es];
    ed_hist = [ed_hist, errors.ed];
    echi_hist = [echi_hist, errors.echi];
    s_hist = [s_hist, errors.s];
    s_dot_hist = [s_dot_hist, errors.s_dot];
    leg_hist = [leg_hist, errors.leg_index];

    fprintf('%7.1f  | %7.2f | %7.2f | %9.4f | %7.2f | %d\n', ...
        t, errors.es, errors.ed, errors.echi, errors.s, errors.leg_index);
end

% ===== 4. 绘图 =====
figure('Name', 'System Model Test', 'Position', [100 100 900 600]);

% 子图1：轨迹对比
subplot(1,2,1);
hold on; grid on;

% 绘制参考路径（第一段直线）
plot([start_lon, end_lon], [start_lat, end_lat], 'b-', 'LineWidth', 3, 'DisplayName', '参考路径');

% 绘制飞机轨迹
plot(trajectory_lon, trajectory_lat, 'r-o', 'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', '飞机轨迹');

% 绘制虚拟目标点
plot(target_lon, target_lat, 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '虚拟目标点(q)');

% 标记起点和终点
plot(start_lon, start_lat, 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'DisplayName', '起点');
plot(end_lon, end_lat, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', '终点');

xlabel('经度 (deg)');
ylabel('纬度 (deg)');
title('轨迹对比');
legend('Location', 'best');

% 子图2：误差变化
subplot(1,2,2);
time_vec = 0:dt:total_time;
hold on; grid on;
plot(time_vec, es_hist, 'b-', 'LineWidth', 2, 'DisplayName', 'es (m)');
plot(time_vec, ed_hist, 'r-', 'LineWidth', 2, 'DisplayName', 'ed (m)');
plot(time_vec, echi_hist, 'k-', 'LineWidth', 2, 'DisplayName', 'echi (rad)');
plot(time_vec, s_dot_hist, 'm--', 'LineWidth', 1.5, 'DisplayName', 's_dot (m/s)');

xlabel('时间 (s)');
ylabel('误差 / 变化率');
title('误差状态变化');
legend('Location', 'best');

% ===== 5. 简单判据 =====
s_is_monotonic = all(diff(s_hist) >= -1e-6);
leg_is_fixed = all(leg_hist == 1);
max_abs_es = max(abs(es_hist));
max_abs_ed = max(abs(ed_hist));
max_abs_echi = max(abs(echi_hist));
final_abs_es = abs(es_hist(end));

fprintf('\n--- 自动判据 ---\n');
fprintf('max|es|   = %.4f m\n', max_abs_es);
fprintf('max|ed|   = %.4f m\n', max_abs_ed);
fprintf('max|echi| = %.6f rad\n', max_abs_echi);
fprintf('|es(end)| = %.4f m\n', final_abs_es);
fprintf('s单调递增: %d\n', s_is_monotonic);
fprintf('航段保持第1段: %d\n', leg_is_fixed);

if s_is_monotonic && leg_is_fixed && final_abs_es < 5 && max_abs_ed < 5 && max_abs_echi < 0.05
    fprintf('结论: 测试通过（横向误差很小，航迹角对齐，es逐步收敛且s单调增加）\n');
else
    fprintf('结论: 测试未通过（请检查误差曲线与阈值）\n');
end

fprintf('\n===== 测试完成 =====\n');
fprintf('图1左: 蓝线=参考路径, 红线=飞机轨迹, 绿三角=虚拟目标点\n');
fprintf('图1右: es/ed/echi与s_dot随时间变化\n');