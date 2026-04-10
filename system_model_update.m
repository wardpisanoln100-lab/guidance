function [sys_model, errors] = system_model_update(sys_model, aircraft_state, current_time)
% system_model_update - 更新水平面系统模型状态

% 飞机状态
aircraft_lat = aircraft_state(1);
aircraft_lon = aircraft_state(2);
aircraft_chi = aircraft_state(3);
aircraft_V = aircraft_state(4);

chi = aircraft_chi * sys_model.deg2rad;
V_chi = aircraft_V;

% 时间步长
dt = current_time - sys_model.last_time;
if dt < 0
    error('system_model_update:NonMonotonicTime', ...
        'current_time 必须大于或等于 sys_model.last_time。');
end

% 更新 s，使 q(s)、chi_f、kappa 与返回误差在当前采样时刻保持一致
cumulative_dist = compute_cumulative_distance(sys_model);
sys_model = update_virtual_target_state( ...
    sys_model, cumulative_dist, aircraft_lat, aircraft_lon, chi, V_chi, dt);

[es, ed, echi] = compute_tracking_errors(sys_model, aircraft_lat, aircraft_lon, chi);
s_dot = sys_model.ks * es + V_chi * cos(echi);

% 回写状态
sys_model.es = es;
sys_model.ed = ed;
sys_model.echi = echi;
sys_model.last_time = current_time;

% 输出
errors.es = es;
errors.ed = ed;
errors.echi = echi;
errors.s = sys_model.s;
errors.leg_index = sys_model.leg_index;
errors.chi_f = sys_model.chi_f;
errors.kappa = sys_model.kappa;
errors.s_dot = s_dot;
errors.s_dot_used = s_dot;
errors.q_lat = sys_model.q_lat;
errors.q_lon = sys_model.q_lon;
end

function sys_model = update_virtual_target_state(sys_model, cumulative_dist, aircraft_lat, aircraft_lon, chi, V_chi, dt)
% update_virtual_target_state - 用不动点迭代更新 s
% 使返回的几何状态与当前飞机采样保持一致

sys_model = sync_virtual_target(sys_model, cumulative_dist);

if dt == 0
    return;
end

s_prev = sys_model.s;
s_candidate = s_prev;
max_iter = 8;
tol = 1e-6;

for iter = 1:max_iter
    trial_model = sys_model;
    trial_model.s = s_candidate;
    trial_model = sync_virtual_target(trial_model, cumulative_dist);

    [es_iter, ~, echi_iter] = compute_tracking_errors( ...
        trial_model, aircraft_lat, aircraft_lon, chi);

    s_next = s_prev + dt * (trial_model.ks * es_iter + V_chi * cos(echi_iter));
    s_next = min(max(s_next, 0), cumulative_dist(end));

    if abs(s_next - s_candidate) < tol
        s_candidate = s_next;
        break;
    end

    s_candidate = s_next;
end

sys_model.s = s_candidate;
sys_model = sync_virtual_target(sys_model, cumulative_dist);
end

function cumulative_dist = compute_cumulative_distance(sys_model)
% compute_cumulative_distance - 计算每个航路点对应的累计路径长度

flight_plan = sys_model.flight_plan;
num_waypoints = size(flight_plan, 1);

cumulative_dist = zeros(1, num_waypoints);
cumulative_dist(1) = 0;

for i = 1:num_waypoints - 1
    leg_row = i + 1;
    leg_type = flight_plan(leg_row, 1);
    start_lat = flight_plan(i, 2);
    start_lon = flight_plan(i, 3);
    end_lat = flight_plan(i + 1, 2);
    end_lon = flight_plan(i + 1, 3);

    if leg_type == 2
        radius = flight_plan(leg_row, 6);
        center_lat = flight_plan(leg_row, 7);
        center_lon = flight_plan(leg_row, 8);
        turn_dir = flight_plan(leg_row, 5);

        if radius > 0
            temp_start = func_RhumbLineInverse(center_lat, center_lon, start_lat, start_lon);
            temp_end = func_RhumbLineInverse(center_lat, center_lon, end_lat, end_lon);
            arc_angle_deg = func_CalculateArcAngle(temp_start(2), temp_end(2), turn_dir);
            leg_length = radius * arc_angle_deg * sys_model.deg2rad;
        else
            leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
            leg_length = leg_data(1);
        end
    else
        leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
        leg_length = leg_data(1);
    end

    cumulative_dist(i + 1) = cumulative_dist(i) + leg_length;
end
end

function sys_model = sync_virtual_target(sys_model, cumulative_dist)
% sync_virtual_target - 让 leg_index 和 q(s) 与当前 s 保持一致

sys_model.s = min(max(sys_model.s, 0), cumulative_dist(end));

while sys_model.leg_index > 1 && sys_model.s < cumulative_dist(sys_model.leg_index)
    sys_model.leg_index = sys_model.leg_index - 1;
end

while sys_model.leg_index < sys_model.num_legs && ...
      sys_model.s >= cumulative_dist(sys_model.leg_index + 1)
    sys_model.leg_index = sys_model.leg_index + 1;
end

local_s = sys_model.s - cumulative_dist(sys_model.leg_index);
[sys_model.q_lat, sys_model.q_lon, sys_model.chi_f, sys_model.kappa] = ...
    compute_virtual_target(sys_model, sys_model.leg_index, local_s);
end

function [es, ed, echi] = compute_tracking_errors(sys_model, aircraft_lat, aircraft_lon, chi)
% compute_tracking_errors - 计算 Serret-Frenet 坐标系下的跟踪误差

temp = func_GreatCircleInverse( ...
    sys_model.q_lat, sys_model.q_lon, ...
    aircraft_lat, aircraft_lon);

dist_q2aircraft = temp(1);
bearing_q2aircraft_rad = temp(2) * sys_model.deg2rad;
chi_f = sys_model.chi_f;

echi = wrapToPi(chi - chi_f);
delta_bearing = wrapToPi(bearing_q2aircraft_rad - chi_f);

% 符号约定：
%   ed < 0 表示飞机位于路径左侧
%   ed > 0 表示飞机位于路径右侧
ed = dist_q2aircraft * sin(delta_bearing);

% es > 0 表示飞机在虚拟目标前方
es = dist_q2aircraft * cos(delta_bearing);
end

function [q_lat, q_lon, chi_f, kappa] = compute_virtual_target(sys_model, leg_index, local_s)
% compute_virtual_target - 计算虚拟目标位置及局部路径几何量

flight_plan = sys_model.flight_plan;
deg2rad = sys_model.deg2rad;
rad2deg = sys_model.rad2deg;

start_lat = flight_plan(leg_index, 2);
start_lon = flight_plan(leg_index, 3);
end_lat = flight_plan(leg_index + 1, 2);
end_lon = flight_plan(leg_index + 1, 3);
leg_row = leg_index + 1;
leg_type = flight_plan(leg_row, 1);

if leg_type == 2
    center_lat = flight_plan(leg_row, 7);
    center_lon = flight_plan(leg_row, 8);
    radius = flight_plan(leg_row, 6);
    turn_dir = flight_plan(leg_row, 5);

    if radius <= 0
        leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
        leg_bearing = leg_data(2);
        leg_length = leg_data(1);
        local_s = min(max(local_s, 0), leg_length);

        q = func_GreatCincleForward(start_lat, start_lon, leg_bearing, local_s);
        q_lat = q(1);
        q_lon = q(2);
        chi_f = wrapTo2Pi(leg_bearing * deg2rad);
        kappa = 0;
        return;
    end

    temp_start = func_RhumbLineInverse(center_lat, center_lon, start_lat, start_lon);
    temp_end = func_RhumbLineInverse(center_lat, center_lon, end_lat, end_lon);
    arc_angle_deg = func_CalculateArcAngle(temp_start(2), temp_end(2), turn_dir);
    arc_length = radius * arc_angle_deg * deg2rad;
    local_s = min(max(local_s, 0), arc_length);

    bearing_c2s = temp_start(2);
    delta_angle_deg = turn_dir * (local_s / radius) * rad2deg;
    bearing_c2q = wrapTo360Deg(bearing_c2s + delta_angle_deg);

    q = func_GreatCincleForward(center_lat, center_lon, bearing_c2q, radius);
    q_lat = q(1);
    q_lon = q(2);
    chi_f = wrapTo2Pi(bearing_c2q * deg2rad + turn_dir * pi / 2);
    kappa = turn_dir / radius;
    return;
end

leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
leg_bearing = leg_data(2);
leg_length = leg_data(1);
local_s = min(max(local_s, 0), leg_length);

q = func_GreatCincleForward(start_lat, start_lon, leg_bearing, local_s);
q_lat = q(1);
q_lon = q(2);
chi_f = wrapTo2Pi(leg_bearing * deg2rad);
kappa = 0;
end

function angle = wrapTo360Deg(angle)
% wrapTo360Deg - 将角度限制到 [0, 360)
angle = mod(angle, 360);
if angle < 0
    angle = angle + 360;
end
end

function angle = wrapTo2Pi(angle)
% wrapTo2Pi - 将角度限制到 [0, 2*pi)
angle = mod(angle, 2 * pi);
if angle < 0
    angle = angle + 2 * pi;
end
end

function angle = wrapToPi(angle)
% wrapToPi - 将角度限制到 [-pi, pi]
angle = mod(angle + pi, 2 * pi) - pi;
end
