% test_system_model.m - 测试 System Model 在直线航段上的表现
clc; clear; close all;
warning off;

fprintf('===== 测试 System Model（直线航段） =====\n\n');

% 1) 加载飞行计划
Manuscript_RNPAR_FlightPlan;
flight_plan = RNPAR_FlightPlan;

% 2) 初始化系统模型
fprintf('--- 初始化系统模型 ---\n');
sys_model = system_model_init(flight_plan);

% 3) 模拟飞机沿第一段直线航段飞行
fprintf('\n--- 仿真过程 ---\n');

start_lat = flight_plan(1, 2);
start_lon = flight_plan(1, 3);
end_lat = flight_plan(2, 2);
end_lon = flight_plan(2, 3);

leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
bearing = leg_data(2);
leg_length = leg_data(1);

fprintf('第一段直线航段：长度 = %.1f m，方位角 = %.2f deg\n', leg_length, bearing);

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

dt = 0.01;
total_time = 12;

fprintf('\n时间(s) | es(m)   | ed(m)   | echi(rad) | s(m)    | 航段\n');
fprintf('-------------------------------------------------------------\n');

for t = 0:dt:total_time
    progress = min(t * 80, leg_length);
    aircraft_pos = func_GreatCincleForward(start_lat, start_lon, bearing, progress);

    aircraft_lat = aircraft_pos(1);
    aircraft_lon = aircraft_pos(2);
    aircraft_chi = bearing;
    aircraft_V = 80;

    aircraft_state = [aircraft_lat, aircraft_lon, aircraft_chi, aircraft_V, 0];
    [sys_model, errors] = system_model_update(sys_model, aircraft_state, t);

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

% 4) 使用与系统模型一致的大圆几何构造参考路径
[ref_lat, ref_lon] = build_reference_line(start_lat, start_lon, bearing, leg_length, 100);

% 5) 绘图
figure('Name', 'System Model Test', 'Position', [100 100 900 600]);

[ref_x, ref_y] = latlon_to_local_xy(ref_lat, ref_lon, start_lat, start_lon);
[traj_x, traj_y] = latlon_to_local_xy(trajectory_lat, trajectory_lon, start_lat, start_lon);
[target_x, target_y] = latlon_to_local_xy(target_lat, target_lon, start_lat, start_lon);
[end_x, end_y] = latlon_to_local_xy(end_lat, end_lon, start_lat, start_lon);

subplot(1,2,1);
hold on; grid on;
plot(ref_x, ref_y, 'b-', 'LineWidth', 3, 'DisplayName', '参考路径');
plot(traj_x, traj_y, 'r-o', 'LineWidth', 2, 'MarkerSize', 6, 'DisplayName', '飞机轨迹');
plot(target_x, target_y, 'g^', 'LineStyle', 'none', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '虚拟目标 q(s)');
plot(0, 0, 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'DisplayName', '起点');
plot(end_x, end_y, 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', '终点');
axis equal;
xlabel('东向位移 (m)');
ylabel('北向位移 (m)');
title('几何轨迹对比（局部平面）');
legend('Location', 'best');

subplot(1,2,2);
time_vec = 0:dt:total_time;
hold on; grid on;
plot(time_vec, es_hist, 'b-', 'LineWidth', 2, 'DisplayName', 'es (m)');
plot(time_vec, ed_hist, 'r-', 'LineWidth', 2, 'DisplayName', 'ed (m)');
plot(time_vec, echi_hist, 'k-', 'LineWidth', 2, 'DisplayName', 'echi (rad)');
plot(time_vec, s_dot_hist, 'm--', 'LineWidth', 1.5, 'DisplayName', 's_dot (m/s)');
xlabel('时间 (s)');
ylabel('误差 / 变化率');
title('误差状态');
legend('Location', 'best');

% 6) 自动检查
s_is_monotonic = all(diff(s_hist) >= -1e-6);
leg_is_fixed = all(leg_hist == 1);
max_abs_es = max(abs(es_hist));
max_abs_ed = max(abs(ed_hist));
max_abs_echi = max(abs(echi_hist));
final_abs_es = abs(es_hist(end));

fprintf('\n--- 自动检查 ---\n');
fprintf('max|es|   = %.4f m\n', max_abs_es);
fprintf('max|ed|   = %.4f m\n', max_abs_ed);
fprintf('max|echi| = %.6f rad\n', max_abs_echi);
fprintf('|es(end)| = %.4f m\n', final_abs_es);
fprintf('s 单调递增：%d\n', s_is_monotonic);
fprintf('航段保持在直线段：%d\n', leg_is_fixed);

if s_is_monotonic && leg_is_fixed && final_abs_es < 5 && max_abs_ed < 5 && max_abs_echi < 0.05
    fprintf('结论：通过\n');
else
    fprintf('结论：失败\n');
end

fprintf('\n===== 测试结束 =====\n');

function [ref_lat, ref_lon] = build_reference_line(start_lat, start_lon, bearing, leg_length, num_points)
ref_lat = zeros(1, num_points + 1);
ref_lon = zeros(1, num_points + 1);

for k = 0:num_points
    s_ref = leg_length * k / num_points;
    p_ref = func_GreatCincleForward(start_lat, start_lon, bearing, s_ref);
    ref_lat(k + 1) = p_ref(1);
    ref_lon(k + 1) = p_ref(2);
end
end

function [x, y] = latlon_to_local_xy(lat, lon, origin_lat, origin_lon)
lat = lat(:).';
lon = lon(:).';
x = zeros(size(lat));
y = zeros(size(lat));

for k = 1:numel(lat)
    geo = func_GreatCircleInverse(origin_lat, origin_lon, lat(k), lon(k));
    dist = geo(1);
    bearing_rad = deg2rad(geo(2));
    x(k) = dist * sin(bearing_rad);
    y(k) = dist * cos(bearing_rad);
end
end
