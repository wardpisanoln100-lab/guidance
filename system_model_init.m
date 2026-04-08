function [sys_model] = system_model_init(flight_plan)
% system_model_init - Initialize the horizontal system model state.

% Constants
sys_model.g = 9.81;
sys_model.R_earth = 6371000;
sys_model.deg2rad = pi / 180;
sys_model.rad2deg = 180 / pi;

% Guidance-law parameters from the paper
sys_model.k = 0.01;
sys_model.ks = 0.2;
sys_model.k_omega = 0.005;
sys_model.gamma = 4000;
sys_model.chi_inf = pi / 2;

% Flight plan
sys_model.flight_plan = flight_plan;
sys_model.num_legs = size(flight_plan, 1) - 1;

% Virtual-target state
sys_model.s = 0;
sys_model.leg_index = 1;
sys_model.q_lat = flight_plan(1, 2);
sys_model.q_lon = flight_plan(1, 3);
sys_model.q_alt = flight_plan(1, 4);

% Path geometry at the initial point
[sys_model.chi_f, sys_model.kappa] = compute_path_geometry(sys_model, 1);

% Tracking errors
sys_model.es = 0;
sys_model.ed = 0;
sys_model.echi = 0;
sys_model.last_time = 0;

fprintf('System Model initialized.\n');
fprintf('  - Initial virtual target: (%.4f, %.4f)\n', sys_model.q_lat, sys_model.q_lon);
fprintf('  - Number of legs: %d\n', sys_model.num_legs);
fprintf('  - Initial path tangent: %.2f deg\n', sys_model.chi_f * sys_model.rad2deg);
fprintf('  - Initial path curvature: %.6f\n', sys_model.kappa);
end

function [chi_f, kappa] = compute_path_geometry(sys_model, leg_index)
% compute_path_geometry - Path tangent and curvature at a leg start.

flight_plan = sys_model.flight_plan;
deg2rad = sys_model.deg2rad;

start_lat = flight_plan(leg_index, 2);
start_lon = flight_plan(leg_index, 3);
end_lat = flight_plan(leg_index + 1, 2);
end_lon = flight_plan(leg_index + 1, 3);
leg_type = flight_plan(leg_index, 1);

if leg_type == 1 || leg_type == 0
    leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
    chi_f = wrapTo2Pi(leg_data(2) * deg2rad);
    kappa = 0;
    return;
end

center_lat = flight_plan(leg_index, 7);
center_lon = flight_plan(leg_index, 8);
radius = flight_plan(leg_index, 6);
turn_dir = flight_plan(leg_index, 5);

if radius <= 0
    leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
    chi_f = wrapTo2Pi(leg_data(2) * deg2rad);
    kappa = 0;
    return;
end

temp_start = func_GreatCircleInverse(center_lat, center_lon, start_lat, start_lon);
bearing_c2start = temp_start(2);
chi_f = wrapTo2Pi(bearing_c2start * deg2rad + turn_dir * pi / 2);
kappa = turn_dir / radius;
end

function angle = wrapTo2Pi(angle)
% wrapTo2Pi - Wrap an angle to [0, 2*pi).
angle = mod(angle, 2 * pi);
if angle < 0
    angle = angle + 2 * pi;
end
end
