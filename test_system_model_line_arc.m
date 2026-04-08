% test_system_model_line_arc.m - Test straight leg + arc leg in one run
clc; clear; close all;
warning off;

fprintf('===== Test System Model (Line + Arc) =====\n\n');

% 1) Load original flight plan
Manuscript_RNPAR_FlightPlan;
flight_plan = RNPAR_FlightPlan;

% 2) Find first straight leg followed by RF leg
line_arc_start_leg = find((flight_plan(1:end-2,1) <= 1) & (flight_plan(2:end-1,1) == 2), 1, 'first');
if isempty(line_arc_start_leg)
    error('Cannot find a straight leg immediately followed by an RF leg.');
end

line_leg = line_arc_start_leg;
arc_leg = line_arc_start_leg + 1;

% 3) Build cumulative distance and key boundaries
cumulative_dist = compute_cumulative_distance_test(flight_plan);
s_start = cumulative_dist(line_leg);
s_switch = cumulative_dist(line_leg + 1);
s_end = cumulative_dist(line_leg + 2);
line_length = s_switch - s_start;
arc_length = s_end - s_switch;
two_leg_length = s_end - s_start;

fprintf('Selected legs: line=%d, arc=%d\n', line_leg, arc_leg);
fprintf('Line length: %.1f m, Arc length: %.1f m, Total: %.1f m\n', line_length, arc_length, two_leg_length);

% 4) Initialize system model at the line-leg start
sys_model = system_model_init(flight_plan);
sys_model.s = s_start;
sys_model.leg_index = line_leg;
sys_model.last_time = 0;

% Geometry for line leg
line_start_lat = flight_plan(line_leg, 2);
line_start_lon = flight_plan(line_leg, 3);
line_end_lat = flight_plan(line_leg + 1, 2);
line_end_lon = flight_plan(line_leg + 1, 3);
line_data = func_GreatCircleInverse(line_start_lat, line_start_lon, line_end_lat, line_end_lon);
line_bearing = line_data(2);

% Geometry for arc leg
arc_start_lat = flight_plan(arc_leg, 2);
arc_start_lon = flight_plan(arc_leg, 3);
arc_end_lat = flight_plan(arc_leg + 1, 2);
arc_end_lon = flight_plan(arc_leg + 1, 3);
arc_center_lat = flight_plan(arc_leg, 7);
arc_center_lon = flight_plan(arc_leg, 8);
arc_radius = flight_plan(arc_leg, 6);
arc_turn_dir = flight_plan(arc_leg, 5);

if arc_radius <= 0
    error('Arc leg radius is invalid at leg %d.', arc_leg);
end

arc_start_data = func_GreatCircleInverse(arc_center_lat, arc_center_lon, arc_start_lat, arc_start_lon);
arc_end_data = func_GreatCircleInverse(arc_center_lat, arc_center_lon, arc_end_lat, arc_end_lon);
arc_bearing_c2s = arc_start_data(2);
arc_angle_deg = func_CalculateArcAngle(arc_start_data(2), arc_end_data(2), arc_turn_dir);
fprintf('Arc radius: %.1f m, Arc angle: %.2f deg\n\n', arc_radius, arc_angle_deg);

% 5) Simulate aircraft moving on line then arc
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

fprintf('Time(s) | es(m)   | ed(m)   | echi(rad) | s(m)     | leg\n');
fprintf('--------------------------------------------------------------\n');

for t = 0:dt:total_time
    s_rel = min(V * t, two_leg_length * 0.95);
    s_abs = s_start + s_rel;

    if s_abs <= s_switch
        % On straight leg
        local_line_s = s_abs - s_start;
        pos = func_GreatCincleForward(line_start_lat, line_start_lon, line_bearing, local_line_s);
        aircraft_lat = pos(1);
        aircraft_lon = pos(2);
        aircraft_chi = line_bearing;
    else
        % On arc leg
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

    if mod(round(t/dt), 25) == 0
        fprintf('%7.2f | %7.2f | %7.2f | %9.4f | %8.2f | %d\n', ...
            t, errors.es, errors.ed, errors.echi, errors.s, errors.leg_index);
    end
end

% 6) Build reference path (line + arc) for plotting
ref_lat = [];
ref_lon = [];

for k = 0:100
    s_line = line_length * k / 100;
    p = func_GreatCincleForward(line_start_lat, line_start_lon, line_bearing, s_line);
    ref_lat = [ref_lat, p(1)];
    ref_lon = [ref_lon, p(2)];
end

for k = 1:100
    s_arc = arc_length * k / 100;
    b = wrap_to_360_deg(arc_bearing_c2s + arc_turn_dir * (s_arc / arc_radius) * 180 / pi);
    p = func_GreatCincleForward(arc_center_lat, arc_center_lon, b, arc_radius);
    ref_lat = [ref_lat, p(1)];
    ref_lon = [ref_lon, p(2)];
end

% 7) Plot
figure('Name', 'System Model Line+Arc Test', 'Position', [100 100 1080 640]);

subplot(1,2,1);
hold on; grid on;
plot(ref_lon, ref_lat, 'b-', 'LineWidth', 3, 'DisplayName', 'Reference path (line+arc)');
plot(aircraft_lon_hist, aircraft_lat_hist, 'r-', 'LineWidth', 2, 'DisplayName', 'Aircraft trajectory');
plot(target_lon_hist, target_lat_hist, 'g^', 'MarkerSize', 5, 'DisplayName', 'Virtual target q(s)');
plot(line_start_lon, line_start_lat, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Line start');
plot(line_end_lon, line_end_lat, 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Line-Arc switch');
plot(arc_end_lon, arc_end_lat, 'kd', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Arc end');
xlabel('Longitude (deg)');
ylabel('Latitude (deg)');
title('Path Geometry');
legend('Location', 'best');

subplot(1,2,2);
time_vec = 0:dt:total_time;
hold on; grid on;
plot(time_vec, es_hist, 'b-', 'LineWidth', 1.8, 'DisplayName', 'es (m)');
plot(time_vec, ed_hist, 'r-', 'LineWidth', 1.8, 'DisplayName', 'ed (m)');
plot(time_vec, echi_hist, 'k-', 'LineWidth', 1.8, 'DisplayName', 'echi (rad)');
plot(time_vec, s_dot_hist, 'm--', 'LineWidth', 1.4, 'DisplayName', 's_dot (m/s)');
xlabel('Time (s)');
ylabel('Error / Rate');
title('Error States');
legend('Location', 'best');

% 8) Automatic checks
s_is_monotonic = all(diff(s_hist) >= -1e-8);
switch_happened = any(leg_hist == arc_leg);
leg_in_range = all((leg_hist >= line_leg) & (leg_hist <= arc_leg));
max_abs_ed = max(abs(ed_hist));
max_abs_echi = max(abs(echi_hist));
final_abs_es = abs(es_hist(end));

fprintf('\n--- Auto checks ---\n');
fprintf('s monotonic increasing: %d\n', s_is_monotonic);
fprintf('line->arc switch happened: %d\n', switch_happened);
fprintf('leg index in expected range: %d\n', leg_in_range);
fprintf('max|ed|   = %.4f m\n', max_abs_ed);
fprintf('max|echi| = %.6f rad\n', max_abs_echi);
fprintf('|es(end)| = %.4f m\n', final_abs_es);

if s_is_monotonic && switch_happened && leg_in_range && max_abs_ed < 8 && max_abs_echi < 0.08 && final_abs_es < 8
    fprintf('Result: PASS (line+arc transition and errors are consistent).\n');
else
    fprintf('Result: FAIL (check transition behavior or thresholds).\n');
end

fprintf('\n===== Test finished =====\n');
fprintf('Left figure: reference path, aircraft trajectory, virtual target.\n');
fprintf('Right figure: es/ed/echi/s_dot over time.\n');

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
