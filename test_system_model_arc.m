% test_system_model_arc.m - System Model圆弧航段测试
clc; clear; close all;
warning off;

fprintf('===== Test System Model (Arc Leg) =====\n\n');

% 1) 加载飞行计划
Manuscript_RNPAR_FlightPlan;
flight_plan = RNPAR_FlightPlan;

% 2) 选择当前实现支持的RF航段
rf_candidates = find(flight_plan(1:end-1, 1) == 2);
if isempty(rf_candidates)
    error('No RF leg found in flight_plan(:,1).');
end
target_leg = rf_candidates(1);

% 3) 初始化System Model并跳转到圆弧段起点
fprintf('--- Initialize System Model ---\n');
sys_model = system_model_init(flight_plan);

cumulative_dist = compute_cumulative_distance_test(flight_plan);
sys_model.s = cumulative_dist(target_leg);
sys_model.leg_index = target_leg;
sys_model.last_time = 0;

% 圆弧几何参数
start_lat = flight_plan(target_leg, 2);
start_lon = flight_plan(target_leg, 3);
end_lat = flight_plan(target_leg + 1, 2);
end_lon = flight_plan(target_leg + 1, 3);
center_lat = flight_plan(target_leg, 7);
center_lon = flight_plan(target_leg, 8);
radius = flight_plan(target_leg, 6);
turn_dir = flight_plan(target_leg, 5);

if radius <= 0
    error('RF leg radius is invalid at leg %d.', target_leg);
end

temp_start = func_GreatCircleInverse(center_lat, center_lon, start_lat, start_lon);
temp_end = func_GreatCircleInverse(center_lat, center_lon, end_lat, end_lon);
bearing_c2s = temp_start(2);
arc_angle_deg = func_CalculateArcAngle(temp_start(2), temp_end(2), turn_dir);
arc_length = radius * arc_angle_deg * pi / 180;

fprintf('RF leg index: %d\n', target_leg);
fprintf('Arc radius: %.1f m, Arc angle: %.2f deg, Arc length: %.1f m\n', radius, arc_angle_deg, arc_length);

% 4) 模拟飞机沿同一圆弧飞行
V = 80;                 % m/s
dt = 0.5;               % s
total_time = min(arc_length / V * 0.9, 60); % 保持在当前圆弧段内

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

    % 圆弧切向方向
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

% 5) 构造用于绘图的参考圆弧
ref_lat = [];
ref_lon = [];
for k = 0:100
    s_ref = arc_length * k / 100;
    b_ref = wrap_to_360_deg(bearing_c2s + turn_dir * (s_ref / radius) * 180 / pi);
    p_ref = func_GreatCincleForward(center_lat, center_lon, b_ref, radius);
    ref_lat = [ref_lat, p_ref(1)];
    ref_lon = [ref_lon, p_ref(2)];
end

% 6) 绘图
figure('Name', 'System Model Arc Test', 'Position', [100 100 980 620]);

subplot(1,2,1);
hold on; grid on;
plot(ref_lon, ref_lat, 'b-', 'LineWidth', 3, 'DisplayName', 'Reference arc');
plot(trajectory_lon, trajectory_lat, 'r-o', 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', 'Aircraft trajectory');
plot(target_lon, target_lat, 'g^', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Virtual target q(s)');
plot(start_lon, start_lat, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'DisplayName', 'Arc start');
plot(end_lon, end_lat, 'bs', 'MarkerSize', 9, 'MarkerFaceColor', 'b', 'DisplayName', 'Arc end');
xlabel('Longitude (deg)');
ylabel('Latitude (deg)');
title('Arc Tracking Geometry');
legend('Location', 'best');

subplot(1,2,2);
time_vec = 0:dt:total_time;
hold on; grid on;
plot(time_vec, es_hist, 'b-', 'LineWidth', 2, 'DisplayName', 'es (m)');
plot(time_vec, ed_hist, 'r-', 'LineWidth', 2, 'DisplayName', 'ed (m)');
plot(time_vec, echi_hist, 'k-', 'LineWidth', 2, 'DisplayName', 'echi (rad)');
plot(time_vec, s_dot_hist, 'm--', 'LineWidth', 1.5, 'DisplayName', 's_dot (m/s)');
xlabel('Time (s)');
ylabel('Error / Rate');
title('Error States on Arc Leg');
legend('Location', 'best');

% 7) 自动判据
s_is_monotonic = all(diff(s_hist) >= -1e-6);
leg_is_fixed = all(leg_hist == target_leg);
max_abs_es = max(abs(es_hist));
max_abs_ed = max(abs(ed_hist));
max_abs_echi = max(abs(echi_hist));
final_abs_es = abs(es_hist(end));

fprintf('\n--- Auto Checks ---\n');
fprintf('max|es|   = %.4f m\n', max_abs_es);
fprintf('max|ed|   = %.4f m\n', max_abs_ed);
fprintf('max|echi| = %.6f rad\n', max_abs_echi);
fprintf('|es(end)| = %.4f m\n', final_abs_es);
fprintf('s monotonic increasing: %d\n', s_is_monotonic);
fprintf('leg fixed at RF leg: %d\n', leg_is_fixed);

if s_is_monotonic && leg_is_fixed && final_abs_es < 8 && max_abs_ed < 8 && max_abs_echi < 0.08
    fprintf('Result: PASS (arc-leg errors remain bounded and states are consistent).\n');
else
    fprintf('Result: FAIL (check curves and thresholds).\n');
end

fprintf('\n===== Arc test finished =====\n');
fprintf('Left: reference arc, aircraft trajectory, virtual target q(s).\n');
fprintf('Right: es/ed/echi and s_dot over time.\n');

function cumulative_dist = compute_cumulative_distance_test(flight_plan)
num_waypoints = size(flight_plan, 1);
cumulative_dist = zeros(1, num_waypoints);

for i = 1:num_waypoints-1
    leg_type = flight_plan(i, 1);
    start_lat = flight_plan(i, 2);
    start_lon = flight_plan(i, 3);
    end_lat = flight_plan(i+1, 2);
    end_lon = flight_plan(i+1, 3);

    if leg_type == 2
        radius = flight_plan(i, 6);
        center_lat = flight_plan(i, 7);
        center_lon = flight_plan(i, 8);
        turn_dir = flight_plan(i, 5);

        if radius > 0
            temp_start = func_GreatCircleInverse(center_lat, center_lon, start_lat, start_lon);
            temp_end = func_GreatCircleInverse(center_lat, center_lon, end_lat, end_lon);
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
