function out = func_GreatCircleForward(start_lat, start_lon, bearing_deg, leg_length)
% func_GreatCircleForward - 大圆航线正算
% 输入:
%   start_lat, start_lon : 起点纬经度 (deg)
%   bearing_deg          : 初始方位角 (deg)
%   leg_length           : 弧长 (m)
% 输出:
%   out = [lat, lon]     : 终点纬经度 (deg)

    rng_rad = leg_length / 6378140;
    bearing_rad = deg2rad(bearing_deg);
    start_lat_rad = deg2rad(start_lat);
    start_lon_rad = deg2rad(start_lon);

    lat_rad = asin(sin(start_lat_rad) * cos(rng_rad) + ...
        cos(start_lat_rad) * sin(rng_rad) * cos(bearing_rad));
    lon_rad = start_lon_rad + atan(sin(rng_rad) * sin(bearing_rad) / ...
        (cos(start_lat_rad) * cos(rng_rad) - ...
        sin(start_lat_rad) * sin(rng_rad) * cos(bearing_rad)));

    out = [rad2deg(lat_rad), rad2deg(lon_rad)];
end