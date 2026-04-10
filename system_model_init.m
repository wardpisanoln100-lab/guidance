function [sys_model] = system_model_init(flight_plan)
% system_model_init - 初始化水平面系统模型状态

% 常量
sys_model.g = 9.81;
sys_model.R_earth = 6371000;
sys_model.deg2rad = pi / 180;
sys_model.rad2deg = 180 / pi;

% 论文中指导律相关参数
sys_model.k = 0.002;
sys_model.ks = 0.002;
sys_model.k_omega = 0.0005;
sys_model.gamma = 12000;
sys_model.chi_inf = pi/2;

% 飞行计划
sys_model.flight_plan = flight_plan;
sys_model.num_legs = size(flight_plan, 1) - 1;

% 虚拟目标状态
sys_model.s = 0;
sys_model.leg_index = 1;
sys_model.q_lat = flight_plan(1, 2);
sys_model.q_lon = flight_plan(1, 3);
sys_model.q_alt = flight_plan(1, 4);

% 初始点处的路径几何量
[sys_model.chi_f, sys_model.kappa] = compute_path_geometry(sys_model, 1);

% 误差状态
sys_model.es = 0;
sys_model.ed = 0;
sys_model.echi = 0;
sys_model.last_time = 0;

fprintf('System Model 初始化完成。\n');
fprintf('  - 初始虚拟目标: (%.4f, %.4f)\n', sys_model.q_lat, sys_model.q_lon);
fprintf('  - 航段数量: %d\n', sys_model.num_legs);
fprintf('  - 初始路径切向角: %.2f deg\n', sys_model.chi_f * sys_model.rad2deg);
fprintf('  - 初始路径曲率: %.6f\n', sys_model.kappa);
end

function [chi_f, kappa] = compute_path_geometry(sys_model, leg_index)
% 计算某一航段起点处的路径切向角和曲率,只是起点处的

flight_plan = sys_model.flight_plan;
deg2rad = sys_model.deg2rad;
leg_row = leg_index + 1;

start_lat = flight_plan(leg_index, 2);
start_lon = flight_plan(leg_index, 3);
end_lat = flight_plan(leg_index + 1, 2);
end_lon = flight_plan(leg_index + 1, 3);
leg_type = flight_plan(leg_row, 1);

if leg_type == 1 || leg_type == 0
	leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
	chi_f = wrapTo2Pi(leg_data(2) * deg2rad);
	kappa = 0;
	return;
end

center_lat = flight_plan(leg_row, 7);
center_lon = flight_plan(leg_row, 8);
radius = flight_plan(leg_row, 6);
turn_dir = flight_plan(leg_row, 5);

if radius <= 0
	leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
	chi_f = wrapTo2Pi(leg_data(2) * deg2rad);
	kappa = 0;
	return;
end

temp_start = func_RhumbLineInverse(center_lat, center_lon, start_lat, start_lon);
bearing_c2start = temp_start(2);
chi_f = wrapTo2Pi(bearing_c2start * deg2rad + turn_dir * pi / 2);
kappa = turn_dir / radius;
end

function angle = wrapTo2Pi(angle)
% wrapTo2Pi - 将角度限制到 [0, 2*pi)
angle = mod(angle, 2 * pi);
if angle < 0
	angle = angle + 2 * pi;
end
end
