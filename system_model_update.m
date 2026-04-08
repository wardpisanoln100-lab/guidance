function [sys_model, errors] = system_model_update(sys_model, aircraft_state, current_time)
% system_model_update - Update the horizontal system model state.

% Aircraft state
aircraft_lat = aircraft_state(1);
aircraft_lon = aircraft_state(2);
aircraft_chi = aircraft_state(3);
aircraft_V = aircraft_state(4);

chi = aircraft_chi * sys_model.deg2rad;
V_chi = aircraft_V;

% Time step
dt = current_time - sys_model.last_time;
if dt < 0
    error('system_model_update:NonMonotonicTime', ...
        'current_time must be greater than or equal to sys_model.last_time.');
end

% Align the virtual target with the current s first.
cumulative_dist = compute_cumulative_distance(sys_model);
sys_model = sync_virtual_target(sys_model, cumulative_dist);

% Use the current error to advance s.
[es, ed, echi] = compute_tracking_errors(sys_model, aircraft_lat, aircraft_lon, chi);
s_dot_used = sys_model.ks * es + V_chi * cos(echi);

if dt > 0
    sys_model.s = sys_model.s + s_dot_used * dt;
    sys_model = sync_virtual_target(sys_model, cumulative_dist);
end

% Return errors that match the updated q(s), chi_f, and kappa.
[es, ed, echi] = compute_tracking_errors(sys_model, aircraft_lat, aircraft_lon, chi);
s_dot = sys_model.ks * es + V_chi * cos(echi);

% Persist state
sys_model.es = es;
sys_model.ed = ed;
sys_model.echi = echi;
sys_model.last_time = current_time;

% Output
errors.es = es;
errors.ed = ed;
errors.echi = echi;
errors.s = sys_model.s;
errors.leg_index = sys_model.leg_index;
errors.chi_f = sys_model.chi_f;
errors.kappa = sys_model.kappa;
errors.s_dot = s_dot;
errors.s_dot_used = s_dot_used;
errors.q_lat = sys_model.q_lat;
errors.q_lon = sys_model.q_lon;
end

function cumulative_dist = compute_cumulative_distance(sys_model)
% compute_cumulative_distance - Cumulative path length at each waypoint.

flight_plan = sys_model.flight_plan;
num_waypoints = size(flight_plan, 1);

cumulative_dist = zeros(1, num_waypoints);
cumulative_dist(1) = 0;

for i = 1:num_waypoints - 1
    leg_type = flight_plan(i, 1);
    start_lat = flight_plan(i, 2);
    start_lon = flight_plan(i, 3);
    end_lat = flight_plan(i + 1, 2);
    end_lon = flight_plan(i + 1, 3);

    if leg_type == 2
        radius = flight_plan(i, 6);
        center_lat = flight_plan(i, 7);
        center_lon = flight_plan(i, 8);
        turn_dir = flight_plan(i, 5);

        if radius > 0
            temp_start = func_GreatCircleInverse(center_lat, center_lon, start_lat, start_lon);
            temp_end = func_GreatCircleInverse(center_lat, center_lon, end_lat, end_lon);
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
% sync_virtual_target - Make leg index and q(s) consistent with s.

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
% compute_tracking_errors - Tracking errors in the Serret-Frenet frame.

temp = func_GreatCircleInverse( ...
    sys_model.q_lat, sys_model.q_lon, ...
    aircraft_lat, aircraft_lon);

dist_q2aircraft = temp(1);
bearing_q2aircraft_rad = temp(2) * sys_model.deg2rad;
chi_f = sys_model.chi_f;

echi = wrapToPi(chi - chi_f);
delta_bearing = wrapToPi(bearing_q2aircraft_rad - chi_f);

% Sign convention:
%   ed < 0 means the aircraft is left of the path.
%   ed > 0 means the aircraft is right of the path.
ed = dist_q2aircraft * sin(delta_bearing);

% Positive es means the aircraft is ahead of the virtual target.
es = dist_q2aircraft * cos(delta_bearing);
end

function [q_lat, q_lon, chi_f, kappa] = compute_virtual_target(sys_model, leg_index, local_s)
% compute_virtual_target - Virtual target and local path geometry.

flight_plan = sys_model.flight_plan;
deg2rad = sys_model.deg2rad;
rad2deg = sys_model.rad2deg;

start_lat = flight_plan(leg_index, 2);
start_lon = flight_plan(leg_index, 3);
end_lat = flight_plan(leg_index + 1, 2);
end_lon = flight_plan(leg_index + 1, 3);
leg_type = flight_plan(leg_index, 1);

if leg_type == 2
    center_lat = flight_plan(leg_index, 7);
    center_lon = flight_plan(leg_index, 8);
    radius = flight_plan(leg_index, 6);
    turn_dir = flight_plan(leg_index, 5);

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

    temp_start = func_GreatCircleInverse(center_lat, center_lon, start_lat, start_lon);
    temp_end = func_GreatCircleInverse(center_lat, center_lon, end_lat, end_lon);
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
% wrapTo360Deg - Wrap an angle to [0, 360).
angle = mod(angle, 360);
if angle < 0
    angle = angle + 360;
end
end

function angle = wrapTo2Pi(angle)
% wrapTo2Pi - Wrap an angle to [0, 2*pi).
angle = mod(angle, 2 * pi);
if angle < 0
    angle = angle + 2 * pi;
end
end

function angle = wrapToPi(angle)
% wrapToPi - Wrap an angle to [-pi, pi].
angle = mod(angle + pi, 2 * pi) - pi;
end
