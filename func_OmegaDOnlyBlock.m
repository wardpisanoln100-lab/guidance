function omega_d = func_OmegaDOnlyBlock(current_time, aircraft_lat, aircraft_lon, aircraft_chi_rad, aircraft_V)
% func_OmegaDOnlyBlock - Simulink 监测支路仅输出 omega_d 的包装函数
%
% 输入顺序与现有监测块保持一致：
%   current_time, aircraft_lat, aircraft_lon, aircraft_chi_rad, aircraft_V
%
% 输出：
%   omega_d (rad/s)

monitor_vec = func_SystemModelMonitorBlock( ...
    current_time, aircraft_lat, aircraft_lon, aircraft_chi_rad, aircraft_V);

omega_d = monitor_vec(11);
end
