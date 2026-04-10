function leg_params = func_PathParameterization(flight_plan, leg_index)
% TF/RF 飞行计划结构的路径参数化
% 将 Y-8 飞行计划格式转换为适用于虚拟目标路径跟踪的参数。
%
% 输入参数:
%   flight_plan : 飞行计划矩阵，每行为:
%                 [航段类型, 纬度, 经度, 高度, 转弯方向, 半径, 圆心纬度, 圆心经度, RNP]
%                 航段类型: 0 = IF, 1 = TF, 2 = RF
%   leg_index   : 当前航段索引 (从 1 开始)
%
% 输出参数:
%   leg_params : 包含以下内容的结构体:
%                - legType: 0 表示 TF, 1 表示 RF
%                - start_lat, start_lon: 起始航路点坐标
%                - end_lat, end_lon: 结束航路点坐标
%                - RF 航段额外包含: center_lat, center_lon, radius, turn_dir, start_angle, end_angle
%                - leg_length: 航段总长度 (m)
%
% 飞行计划格式 (来自 Manuscript_RNPAR_FlightPlan.m):
%   第 1 列: 航段类型 (0=IF, 1=TF, 2=RF)
%   第 2 列: 纬度 (度)
%   第 3 列: 经度 (度)
%   第 4 列: 高度 (m)
%   第 5 列: 转弯方向 (-1=左转/逆时针, 1=右转/顺时针) 用于 RF 航段
%   第 6 列: 转弯半径 (m) 用于 RF 航段
%   第 7 列: 圆心纬度 (度) 用于 RF 航段
%   第 8 列: 圆心经度 (度) 用于 RF 航段
%   第 9 列: RNP 要求

% 常数定义
R_earth = 6371000;
deg_to_rad = pi / 180;

% 初始化输出
leg_params = struct();

% 获取当前和下一个航路点
current_wp = flight_plan(leg_index, :);
next_wp_idx = leg_index + 1;

if next_wp_idx > size(flight_plan, 1)
    % 没有下一个航路点 - 这是最后一段
    % 使用当前航路点作为终点
    next_wp = current_wp;
else
    next_wp = flight_plan(next_wp_idx, :);
end

% 提取航段类型 (转换: 0=IF, 1=TF, 2=RF -> 0=TF, 1=RF)
leg_type_raw = current_wp(1);
if leg_type_raw == 1
    leg_params.legType = 0;  % TF 航段
elseif leg_type_raw == 2
    leg_params.legType = 1;  % RF 航段
else
    leg_params.legType = 0;  % IF 航段默认为 TF
end

% 起始航路点 (TF 航段使用前一个航路点)
if leg_index == 1
    % 第一段 - 使用第一个航路点作为起点
    leg_params.start_lat = flight_plan(1, 2);
    leg_params.start_lon = flight_plan(1, 3);
else
    prev_wp = flight_plan(leg_index - 1, :);
    leg_params.start_lat = prev_wp(2);
    leg_params.start_lon = prev_wp(3);
end

% 结束航路点
leg_params.end_lat = next_wp(2);
leg_params.end_lon = next_wp(3);

% 计算航段方向和长度
d_lat = (leg_params.end_lat - leg_params.start_lat) * deg_to_rad * R_earth;
d_lon = (leg_params.end_lon - leg_params.start_lon) * deg_to_rad * R_earth * cos(leg_params.start_lat * deg_to_rad);
leg_params.leg_length = sqrt(d_lat^2 + d_lon^2);
leg_params.chi_f = atan2(d_lon, d_lat);  % 参考方位角

% RF 航段特定参数
if leg_params.legType == 1
    % 从飞行计划提取 RF 参数
    leg_params.turn_dir = current_wp(5);   % -1 = 逆时针/左转, 1 = 顺时针/右转
    leg_params.radius = current_wp(6);     % 转弯半径 (米)
    leg_params.center_lat = current_wp(7); % 圆心纬度
    leg_params.center_lon = current_wp(8); % 圆心经度

    % 计算圆上的起始和结束角度
    % 起始角度: 从圆心到起始航路点的角度
    d_lat_start = (leg_params.start_lat - leg_params.center_lat) * deg_to_rad * R_earth;
    d_lon_start = (leg_params.start_lon - leg_params.center_lon) * deg_to_rad * R_earth * cos(leg_params.center_lat * deg_to_rad);
    leg_params.start_angle = atan2(d_lon_start, d_lat_start);  % 从北向的角度

    % 结束角度: 从圆心到结束航路点的角度
    d_lat_end = (leg_params.end_lat - leg_params.center_lat) * deg_to_rad * R_earth;
    d_lon_end = (leg_params.end_lon - leg_params.center_lon) * deg_to_rad * R_earth * cos(leg_params.center_lat * deg_to_rad);
    leg_params.end_angle = atan2(d_lon_end, d_lat_end);

    % 根据转弯方向调整结束角度
    if leg_params.turn_dir > 0  % 顺时针 (右转)
        % 确保顺时针转弯时 end_angle < start_angle
        while leg_params.end_angle > leg_params.start_angle
            leg_params.end_angle = leg_params.end_angle - 2*pi;
        end
    else  % 逆时针 (左转)
        % 确保逆时针转弯时 end_angle > start_angle
        while leg_params.end_angle < leg_params.start_angle
            leg_params.end_angle = leg_params.end_angle + 2*pi;
        end
    end

    % 计算弧长
    leg_params.arc_length = abs(leg_params.end_angle - leg_params.start_angle) * leg_params.radius;
end

% 高度剖面
leg_params.start_alt = current_wp(4);
leg_params.end_alt = next_wp(4);

end