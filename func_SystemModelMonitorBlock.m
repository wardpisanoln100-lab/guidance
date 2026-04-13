function y = func_SystemModelMonitorBlock(current_time, aircraft_lat, aircraft_lon, aircraft_chi_rad, aircraft_V, aircraft_phi)
% func_SystemModelMonitorBlock - 供 Simulink 监测支路调用的系统模型包装函数
%
% 输出顺序：
% [q_lat; q_lon; es; ed; echi; s; leg_index; chi_f; kappa; s_dot;
%  omega_d; phi_c; lambda_phi_hat; omega_e]
%
% 输入 (单位):
%   aircraft_phi - 当前滚转角，单位: rad (用于反步法计算)

persistent sys_model lambda_phi_hat omega_d_prev

y = nan(14, 1);

if isempty(sys_model) || current_time <= 0 || ...
        (isfield(sys_model, 'last_time') && current_time < sys_model.last_time)
    flight_plan = get_monitor_flight_plan();
    sys_model = system_model_init(flight_plan);
    lambda_phi_hat = sys_model.lambda_phi_hat;
    omega_d_prev = [];
end

aircraft_chi_deg = aircraft_chi_rad * 180 / pi;
aircraft_state = [aircraft_lat, aircraft_lon, aircraft_chi_deg, aircraft_V, 0];

% 保存上一次的时间（用于计算 dt）
dt = current_time - sys_model.last_time;

[sys_model, errors] = system_model_update(sys_model, aircraft_state, current_time);

% --- 运动学控制律 (Eq.5, Eq.6a, Eq.6b) ---
params = struct( ...
    'ks', sys_model.ks, ...
    'k_omega', sys_model.k_omega, ...
    'gamma', sys_model.gamma, ...
    'k', sys_model.k, ...
    'chi_inf', sys_model.chi_inf);
[omega_d, ~, detail] = func_KinematicControlLaw( ...
    errors.es, errors.ed, errors.echi, aircraft_V, errors.kappa, params);

% --- 计算 omega_d_dot (有限差分) ---
if isempty(omega_d_prev) || dt <= 0
    omega_d_dot = 0;
else
    omega_d_dot = (omega_d - omega_d_prev) / dt;
    % 限幅防止数值噪声
    omega_d_dot_max = 5.0;  % rad/s^2
    omega_d_dot = max(-omega_d_dot_max, min(omega_d_dot_max, omega_d_dot));
end
omega_d_prev = omega_d;

% --- 反步法滚转指令 (Eq.7-11) — 先计算 omega_e ---
params_back = struct( ...
    'k_e', sys_model.k_e, ...
    'g', sys_model.g, ...
    'lambda_phi_hat', lambda_phi_hat);
[~, ~, omega_e, ~] = func_BacksteppingRollCommand( ...
    aircraft_phi, omega_d, omega_d_dot, errors.echi, detail.delta, ...
    aircraft_V, params_back);

% --- 自适应参数估计 (Eq.24) ---
params_adapt = struct('k_a', sys_model.k_a);
[lambda_phi_hat, ~] = func_AdaptiveParameterEstimation( ...
    lambda_phi_hat, omega_e, errors.echi, detail.delta, omega_d_dot, ...
    params_adapt, dt);

% --- 反步法滚转指令 — 用更新后的 lambda_hat 重新计算 phi_c ---
params_back.lambda_phi_hat = lambda_phi_hat;
[phi_c, ~, omega_e, ~] = func_BacksteppingRollCommand( ...
    aircraft_phi, omega_d, omega_d_dot, errors.echi, detail.delta, ...
    aircraft_V, params_back);

% --- 输出赋值 ---
y(1)  = errors.q_lat;
y(2)  = errors.q_lon;
y(3)  = errors.es;
y(4)  = errors.ed;
y(5)  = errors.echi;
y(6)  = errors.s;
y(7)  = errors.leg_index;
y(8)  = errors.chi_f;
y(9)  = errors.kappa;
y(10) = errors.s_dot;
y(11) = omega_d;
y(12) = phi_c;
y(13) = lambda_phi_hat;
y(14) = omega_e;
end

function flight_plan = get_monitor_flight_plan()
if evalin('base', 'exist(''RNPAR_FlightPlan'', ''var'')')
    flight_plan = evalin('base', 'RNPAR_FlightPlan');
elseif evalin('base', 'exist(''system_model_monitor_flight_plan'', ''var'')')
    flight_plan = evalin('base', 'system_model_monitor_flight_plan');
else
    error('func_SystemModelMonitorBlock:MissingFlightPlan', ...
        '基工作区缺少 RNPAR_FlightPlan。请先运行 Start.m。');
end
end
