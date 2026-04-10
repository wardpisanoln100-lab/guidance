% test_system_model_line_arc.m - 测试直线航段衔接圆弧航段
clc; clear; close all;
warning off;

fprintf('===== 测试 System Model（直线 + 圆弧） =====\n\n');

% 1) 加载飞行计划
Manuscript_RNPAR_FlightPlan;
flight_plan = RNPAR_FlightPlan;

% 2) 找到第一组“直线段后接 RF 段”的航段组合（第 i 段对应 flight_plan(i+1,:)）
leg_types = flight_plan(2:end, 1);
line_arc_start_leg = find((leg_types(1:end-1) <= 1) & (leg_types(2:end) == 2), 1, 'first');
if isempty(line_arc_start_leg)
    error('Cannot find a straight leg immediately followed by an RF leg.');
end

line_leg = line_arc_start_leg;
arc_leg = line_arc_start_leg + 1;

% 3) 计算累计距离和关键弧长边界
cumulative_dist = compute_cumulative_distance_test(flight_plan);
s_start = cumulative_dist(line_leg);
s_switch = cumulative_dist(line_leg + 1);
s_end = cumulative_dist(line_leg + 2);
line_length = s_switch - s_start;
arc_length = s_end - s_switch;
two_leg_length = s_end - s_start;

fprintf('Selected legs: line=%d, arc=%d\n', line_leg, arc_leg);
fprintf('Line length: %.1f m, Arc length: %.1f m, Total: %.1f m\n', line_length, arc_length, two_leg_length);

% 4) 在直线段起点初始化系统模型
sys_model = system_model_init(flight_plan);
sys_model.s = s_start;
sys_model.leg_index = line_leg;
sys_model.last_time = 0;

% 直线段几何
line_start_lat = flight_plan(line_leg, 2);
line_start_lon = flight_plan(line_leg, 3);
line_end_lat = flight_plan(line_leg + 1, 2);
line_end_lon = flight_plan(line_leg + 1, 3);
line_data = func_GreatCircleInverse(line_start_lat, line_start_lon, line_end_lat, line_end_lon);
line_bearing = line_data(2);

% 圆弧段几何
arc_start_lat = flight_plan(arc_leg, 2);
arc_start_lon = flight_plan(arc_leg, 3);
arc_end_lat = flight_plan(arc_leg + 1, 2);
arc_end_lon = flight_plan(arc_leg + 1, 3);
arc_leg_row = arc_leg + 1;
arc_center_lat = flight_plan(arc_leg_row, 7);
arc_center_lon = flight_plan(arc_leg_row, 8);
arc_radius = flight_plan(arc_leg_row, 6);
arc_turn_dir = flight_plan(arc_leg_row, 5);

if arc_radius <= 0
    error('Arc leg radius is invalid at leg %d.', arc_leg);
end

arc_start_data = func_RhumbLineInverse(arc_center_lat, arc_center_lon, arc_start_lat, arc_start_lon);
arc_end_data = func_RhumbLineInverse(arc_center_lat, arc_center_lon, arc_end_lat, arc_end_lon);
arc_bearing_c2s = arc_start_data(2);
arc_angle_deg = func_CalculateArcAngle(arc_start_data(2), arc_end_data(2), arc_turn_dir);
fprintf('Arc radius: %.1f m, Arc angle: %.2f deg\n\n', arc_radius, arc_angle_deg);

% 5) 模拟飞机先沿直线飞行，再转入圆弧
V = 80;
dt = 0.02;
total_time = min(two_leg_length / V * 0.95, 120);

aircraft_lat_hist = [];
aircraft_lon_hist = [];
target_lat_hist = [];
target_lon_hist = [];
es_hist = [];
ed_hist = [];
echi_hist = [];
s_hist = [];
s_dot_hist = [];
leg_hist = [];

fprintf('时间(s) | es(m)   | ed(m)   | echi(rad) | s(m)     | 航段\n');
fprintf('--------------------------------------------------------------\n');

for t = 0:dt:total_time
    s_rel = min(V * t, two_leg_length * 0.95);
    s_abs = s_start + s_rel;

    if s_abs <= s_switch
        local_line_s = s_abs - s_start;
        pos = func_GreatCincleForward(line_start_lat, line_start_lon, line_bearing, local_line_s);
        aircraft_lat = pos(1);
        aircraft_lon = pos(2);
        aircraft_chi = line_bearing;
    else
        local_arc_s = s_abs - s_switch;
        bearing_c2air = wrap_to_360_deg(arc_bearing_c2s + arc_turn_dir * (local_arc_s / arc_radius) * 180 / pi);
        pos = func_GreatCincleForward(arc_center_lat, arc_center_lon, bearing_c2air, arc_radius);
        aircraft_lat = pos(1);
        aircraft_lon = pos(2);
        aircraft_chi = wrap_to_360_deg(bearing_c2air + arc_turn_dir * 90);
    end

    aircraft_state = [aircraft_lat, aircraft_lon, aircraft_chi, V, 0];
    [sys_model, errors] = system_model_update(sys_model, aircraft_state, t);

    aircraft_lat_hist = [aircraft_lat_hist, aircraft_lat];
    aircraft_lon_hist = [aircraft_lon_hist, aircraft_lon];
    target_lat_hist = [target_lat_hist, errors.q_lat];
    target_lon_hist = [target_lon_hist, errors.q_lon];
    es_hist = [es_hist, errors.es];
    ed_hist = [ed_hist, errors.ed];
    echi_hist = [echi_hist, errors.echi];
    s_hist = [s_hist, errors.s];
    s_dot_hist = [s_dot_hist, errors.s_dot];
    leg_hist = [leg_hist, errors.leg_index];

    if mod(round(t / dt), 25) == 0
        fprintf('%7.2f | %7.2f | %7.2f | %9.4f | %8.2f | %d\n', ...
            t, errors.es, errors.ed, errors.echi, errors.s, errors.leg_index);
    end
end

% 6) 使用与系统模型一致的几何构造参考路径
[ref_lat_line, ref_lon_line] = build_reference_line(line_start_lat, line_start_lon, line_bearing, line_length, 100);
[ref_lat_arc, ref_lon_arc] = build_reference_arc(arc_center_lat, arc_center_lon, arc_bearing_c2s, arc_radius, arc_turn_dir, arc_length, 100);
ref_lat = [ref_lat_line, ref_lat_arc(2:end)];
ref_lon = [ref_lon_line, ref_lon_arc(2:end)];

% 7) 绘图
figure('Name', 'System Model Line+Arc Test', 'Position', [100 100 1080 640]);

origin_lat = line_start_lat;
origin_lon = line_start_lon;
[ref_x, ref_y] = latlon_to_local_xy(ref_lat, ref_lon, origin_lat, origin_lon);
[aircraft_x, aircraft_y] = latlon_to_local_xy(aircraft_lat_hist, aircraft_lon_hist, origin_lat, origin_lon);
[target_x, target_y] = latlon_to_local_xy(target_lat_hist, target_lon_hist, origin_lat, origin_lon);
[switch_x, switch_y] = latlon_to_local_xy(line_end_lat, line_end_lon, origin_lat, origin_lon);
[arc_end_x, arc_end_y] = latlon_to_local_xy(arc_end_lat, arc_end_lon, origin_lat, origin_lon);

subplot(1,2,1);
hold on; grid on;
plot(ref_x, ref_y, 'b-', 'LineWidth', 3, 'DisplayName', '参考路径（直线+圆弧）');
plot(aircraft_x, aircraft_y, 'r-', 'LineWidth', 2, 'DisplayName', '飞机轨迹');
plot(target_x, target_y, 'g^', 'LineStyle', 'none', 'MarkerSize', 5, 'DisplayName', '虚拟目标 q(s)');
plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', '直线起点');
plot(switch_x, switch_y, 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', '直线-圆弧切换点');
plot(arc_end_x, arc_end_y, 'kd', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', '圆弧终点');
axis equal;
xlabel('东向位移 (m)');
ylabel('北向位移 (m)');
title('路径几何（局部平面）');
legend('Location', 'best');

subplot(1,2,2);
time_vec = 0:dt:total_time;
hold on; grid on;
plot(time_vec, es_hist, 'b-', 'LineWidth', 1.8, 'DisplayName', 'es (m)');
plot(time_vec, ed_hist, 'r-', 'LineWidth', 1.8, 'DisplayName', 'ed (m)');
plot(time_vec, echi_hist, 'k-', 'LineWidth', 1.8, 'DisplayName', 'echi (rad)');
plot(time_vec, s_dot_hist, 'm--', 'LineWidth', 1.4, 'DisplayName', 's_dot (m/s)');
xlabel('时间 (s)');
ylabel('误差 / 变化率');
title('误差状态');
legend('Location', 'best');

% 8) 自动检查
s_is_monotonic = all(diff(s_hist) >= -1e-8);
switch_happened = any(leg_hist == arc_leg);
leg_in_range = all((leg_hist >= line_leg) & (leg_hist <= arc_leg));
max_abs_ed = max(abs(ed_hist));
max_abs_echi = max(abs(echi_hist));
final_abs_es = abs(es_hist(end));

fprintf('\n--- 自动检查 ---\n');
fprintf('s 单调递增：%d\n', s_is_monotonic);
fprintf('直线到圆弧切换发生：%d\n', switch_happened);
fprintf('航段索引在预期范围内：%d\n', leg_in_range);
fprintf('max|ed|   = %.4f m\n', max_abs_ed);
fprintf('max|echi| = %.6f rad\n', max_abs_echi);
fprintf('|es(end)| = %.4f m\n', final_abs_es);

if s_is_monotonic && switch_happened && leg_in_range && max_abs_ed < 8 && max_abs_echi < 0.08 && final_abs_es < 8
    fprintf('结论：通过（直线到圆弧切换与误差状态一致）\n');
else
    fprintf('结论：失败（请检查切换行为或阈值）\n');
end

fprintf('\n===== 测试结束 =====\n');

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
