function setup_afcs_system_model_test()
% setup_afcs_system_model_test - 为 AFCS 全飞行计划测试准备初始条件和建议仿真时长

if ~evalin('base', 'exist(''RNPAR_FlightPlan'', ''var'')')
    evalin('base', 'Manuscript_RNPAR_FlightPlan;');
end

flight_plan = evalin('base', 'RNPAR_FlightPlan');

if evalin('base', 'exist(''Vt'', ''var'')')
    Vt = evalin('base', 'Vt');
else
    Vt = 80;
end

init_lat = flight_plan(1, 2);
init_lon = flight_plan(1, 3);
init_alt = flight_plan(1, 4);

first_leg_data = func_GreatCircleInverse( ...
    flight_plan(1, 2), flight_plan(1, 3), ...
    flight_plan(2, 2), flight_plan(2, 3));
init_heading = first_leg_data(2) * pi / 180;

cumulative_dist = compute_flight_plan_cumulative_distance(flight_plan);
recommended_stop_time = ceil(cumulative_dist(end) / max(Vt, 1) * 1.2);

assignin('base', 'Initi_Lati', init_lat);
assignin('base', 'Initi_Longi', init_lon);
assignin('base', 'alt', init_alt);
assignin('base', 'Initi_Heading', init_heading);
assignin('base', 'afcs_system_model_stop_time', recommended_stop_time);

evalin('base', 'AircraftTrim;');
clear func_SystemModelMonitorBlock system_model_init system_model_update

disp('AFCS 系统模型测试条件已准备完成。');
fprintf('  - 初始纬度: %.4f deg\n', init_lat);
fprintf('  - 初始经度: %.4f deg\n', init_lon);
fprintf('  - 初始航向: %.2f deg\n', init_heading * 180 / pi);
fprintf('  - 建议仿真时长: %.1f s\n', recommended_stop_time);
end

function cumulative_dist = compute_flight_plan_cumulative_distance(flight_plan)
num_waypoints = size(flight_plan, 1);
cumulative_dist = zeros(1, num_waypoints);

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
            leg_length = radius * arc_angle_deg * pi / 180;
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
