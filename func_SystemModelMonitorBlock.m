function y = func_SystemModelMonitorBlock(current_time, aircraft_lat, aircraft_lon, aircraft_chi_rad, aircraft_V)
% func_SystemModelMonitorBlock - 供 Simulink 监测支路调用的系统模型包装函数
%
% 输出顺序：
% [q_lat; q_lon; es; ed; echi; s; leg_index; chi_f; kappa; s_dot; omega_d]

persistent sys_model

y = nan(11, 1);

if isempty(sys_model) || current_time <= 0 || ...
        (isfield(sys_model, 'last_time') && current_time < sys_model.last_time)
    flight_plan = get_monitor_flight_plan();
    sys_model = system_model_init(flight_plan);
end

aircraft_chi_deg = aircraft_chi_rad * 180 / pi;
aircraft_state = [aircraft_lat, aircraft_lon, aircraft_chi_deg, aircraft_V, 0];

[sys_model, errors] = system_model_update(sys_model, aircraft_state, current_time);

params = struct( ...
    'ks', sys_model.ks, ...
    'k_omega', sys_model.k_omega, ...
    'gamma', sys_model.gamma, ...
    'k', sys_model.k, ...
    'chi_inf', sys_model.chi_inf);
[omega_d, ~] = func_KinematicControlLaw( ...
    errors.es, errors.ed, errors.echi, aircraft_V, errors.kappa, params);

y(1) = errors.q_lat;
y(2) = errors.q_lon;
y(3) = errors.es;
y(4) = errors.ed;
y(5) = errors.echi;
y(6) = errors.s;
y(7) = errors.leg_index;
y(8) = errors.chi_f;
y(9) = errors.kappa;
y(10) = errors.s_dot;
y(11) = omega_d;
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
