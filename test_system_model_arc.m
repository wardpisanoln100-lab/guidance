% test_system_model_arc.m - 测试 System Model 在 RF 圆弧航段上的表现
clc; clear; close all;
warning off;

fprintf('===== 测试 System Model（圆弧航段） =====\n\n');

% 1) 加载飞行计划
Manuscript_RNPAR_FlightPlan;
flight_plan = RNPAR_FlightPlan;

% 2) 找到第一个 RF 航段（第 i 段对应 flight_plan(i+1,:)）
rf_leg_rows = find(flight_plan(2:end, 1) == 2) + 1;
if isempty(rf_leg_rows)
    error('No RF leg found in flight_plan(:,1).');
end
target_leg = rf_leg_rows(1) - 1;
target_leg_row = target_leg + 1;

% 3) 初始化系统模型，并跳转到圆弧段起点
fprintf('--- 初始化系统模型 ---\n');
sys_model = system_model_init(flight_plan);

cumulative_dist = compute_cumulative_distance_test(flight_plan);
sys_model.s = cumulative_dist(target_leg);
sys_model.leg_index = target_leg;
sys_model.last_time = 0;

start_lat = flight_plan(target_leg, 2);
start_lon = flight_plan(target_leg, 3);
end_lat = flight_plan(target_leg + 1, 2);
end_lon = flight_plan(target_leg + 1, 3);
center_lat = flight_plan(target_leg_row, 7);
center_lon = flight_plan(target_leg_row, 8);
radius = flight_plan(target_leg_row, 6);
turn_dir = flight_plan(target_leg_row, 5);

if radius <= 0
    error('RF leg radius is invalid at leg %d.', target_leg);
end

temp_start = func_RhumbLineInverse(center_lat, center_lon, start_lat, start_lon);
temp_end = func_RhumbLineInverse(center_lat, center_lon, end_lat, end_lon);
bearing_c2s = temp_start(2);
arc_angle_deg = func_CalculateArcAngle(temp_start(2), temp_end(2), turn_dir);
arc_length = radius * arc_angle_deg * pi / 180;

fprintf('RF leg index: %d\n', target_leg);
fprintf('Arc radius: %.1f m, Arc angle: %.2f deg, Arc length: %.1f m\n', radius, arc_angle_deg, arc_length);

% 4) 模拟飞机沿同一圆弧飞行
V = 80;
dt = 0.5;
total_time = min(arc_length / V * 0.9, 60);

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

fprintf('\nTime(s) | es(m)   | ed(m)   | echi(rad) | s(m)    | Leg\n');
fprintf('-----------------------------------------------------------\n');

for t = 0:dt:total_time
    local_s = min(V * t, arc_length * 0.95);
    bearing_c2air = wrap_to_360_deg(bearing_c2s + turn_dir * (local_s / radius) * 180 / pi);

    aircraft_pos = func_GreatCincleForward(center_lat, center_lon, bearing_c2air, radius);
    aircraft_lat = aircraft_pos(1);
    aircraft_lon = aircraft_pos(2);
    aircraft_chi = wrap_to_360_deg(bearing_c2air + turn_dir * 90);

    aircraft_state = [aircraft_lat, aircraft_lon, aircraft_chi, V, 0];
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

    fprintf('%7.1f | %7.2f | %7.2f | %9.4f | %7.2f | %d\n', ...
        t, errors.es, errors.ed, errors.echi, errors.s, errors.leg_index);
end

% 5) 使用与系统模型一致的几何构造参考圆弧
[ref_lat, ref_lon] = build_reference_arc(center_lat, center_lon, bearing_c2s, radius, turn_dir, arc_length, 100);

% 6) 绘图
figure('Name', 'System Model Arc Test', 'Position', [100 100 980 620]);

[ref_x, ref_y] = latlon_to_local_xy(ref_lat, ref_lon, start_lat, start_lon);
[traj_x, traj_y] = latlon_to_local_xy(trajectory_lat, trajectory_lon, start_lat, start_lon);
[target_x, target_y] = latlon_to_local_xy(target_lat, target_lon, start_lat, start_lon);
[end_x, end_y] = latlon_to_local_xy(end_lat, end_lon, start_lat, start_lon);

subplot(1,2,1);
hold on; grid on;
plot(ref_x, ref_y, 'b-', 'LineWidth', 3, 'DisplayName', '参考圆弧');
plot(traj_x, traj_y, 'r-o', 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', '飞机轨迹');
plot(target_x, target_y, 'g^', 'LineStyle', 'none', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', '虚拟目标 q(s)');
plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'DisplayName', '圆弧起点');
plot(end_x, end_y, 'bs', 'MarkerSize', 9, 'MarkerFaceColor', 'b', 'DisplayName', '圆弧终点');
axis equal;
xlabel('东向位移 (m)');
ylabel('北向位移 (m)');
title('圆弧航迹几何（局部平面）');
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
title('圆弧航段误差状态');
legend('Location', 'best');

% 7) 自动检查
s_is_monotonic = all(diff(s_hist) >= -1e-6);
leg_is_fixed = all(leg_hist == target_leg);
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
fprintf('航段保持在 RF 段：%d\n', leg_is_fixed);

if s_is_monotonic && leg_is_fixed && final_abs_es < 8 && max_abs_ed < 8 && max_abs_echi < 0.08
    fprintf('结论：通过（圆弧段误差有界且状态一致）\n');
else
    fprintf('结论：失败（请检查曲线或阈值）\n');
end

fprintf('\n===== 圆弧测试结束 =====\n');

function cumulative_dist = compute_cumulative_distance_test(flight_plan)
num_waypoints = size(flight_plan, 1);
cumulative_dist = zeros(1, num_waypoints);

for i = 1:num_waypoints-1
    leg_row = i + 1;
    leg_type = flight_plan(leg_row, 1);
    start_lat = flight_plan(i, 2);
    start_lon = flight_plan(i, 3);
    end_lat = flight_plan(i+1, 2);
    end_lon = flight_plan(i+1, 3);

    if leg_type == 2
        radius = flight_plan(leg_row, 6);
        center_lat = flight_plan(leg_row, 7);
        center_lon = flight_plan(leg_row, 8);
        turn_dir = flight_plan(leg_row, 5);

        if radius > 0
            temp_start = func_RhumbLineInverse(center_lat, center_lon, start_lat, start_lon);
            temp_end = func_RhumbLineInverse(center_lat, center_lon, end_lat, end_lon);
            arc_angle_deg = func_CalculateArcAngle(temp_start(2), temp_end(2), turn_dir);
            leg_length = radius * arc_angle_deg * pi / 180;
        else
            leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
            leg_length = leg_data(1);
        end
    else
        leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
        leg_length = leg_data(1);
    end

    cumulative_dist(i+1) = cumulative_dist(i) + leg_length;
end
end

function angle = wrap_to_360_deg(angle)
angle = mod(angle, 360);
if angle < 0
    angle = angle + 360;
end
end

function [ref_lat, ref_lon] = build_reference_arc(center_lat, center_lon, bearing_c2s, radius, turn_dir, arc_length, num_points)
ref_lat = zeros(1, num_points + 1);
ref_lon = zeros(1, num_points + 1);

for k = 0:num_points
    s_ref = arc_length * k / num_points;
    b_ref = wrap_to_360_deg(bearing_c2s + turn_dir * (s_ref / radius) * 180 / pi);
    p_ref = func_GreatCincleForward(center_lat, center_lon, b_ref, radius);
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
