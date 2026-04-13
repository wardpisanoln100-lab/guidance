function plot_afcs_system_model_results()
% plot_afcs_system_model_results
% 参考 Manuscript_RNPAR_Figure.m 的经纬度绘图方式，
% 绘制参考路径、飞机轨迹、虚拟目标 q(s) 以及误差状态。

Font_name = 'Times New Roman';
Font_size = 14;

if ~base_var_exists('RNPAR_FlightPlan')
    evalin('base', 'Manuscript_RNPAR_FlightPlan;');
end

flight_plan = evalin('base', 'RNPAR_FlightPlan');
[q_lat, q_lon, es, ed, echi, s, leg_index, chi_f, kappa, s_dot, ...
    omega_d, phi_c, lambda_phi_hat, omega_e] = get_monitor_outputs();
[plane_lat, plane_lon, has_plane_track] = get_plane_track();

num_samples = min([ ...
    numel(q_lat), numel(q_lon), numel(es), numel(ed), numel(echi), ...
    numel(s), numel(leg_index), numel(chi_f), numel(kappa), numel(s_dot), ...
    numel(omega_d), numel(phi_c), numel(lambda_phi_hat), numel(omega_e)]);

if has_plane_track
    num_samples = min(num_samples, numel(plane_lat));
    num_samples = min(num_samples, numel(plane_lon));
    plane_lat = plane_lat(1:num_samples);
    plane_lon = plane_lon(1:num_samples);
end

q_lat = q_lat(1:num_samples);
q_lon = q_lon(1:num_samples);
es = es(1:num_samples);
ed = ed(1:num_samples);
echi = echi(1:num_samples);
s = s(1:num_samples);
leg_index = leg_index(1:num_samples);
chi_f = chi_f(1:num_samples);
kappa = kappa(1:num_samples);
omega_d = omega_d(1:num_samples);
phi_c = phi_c(1:num_samples);
lambda_phi_hat = lambda_phi_hat(1:num_samples);
omega_e = omega_e(1:num_samples);

time_vec = get_time_vector(num_samples);
time_vec = time_vec(1:num_samples);

q_step = hypot(diff(q_lat), diff(q_lon));
q_is_moving = any(q_step > 1e-10);

fprintf('===== AFCS + System Model 检查结果 =====\n');
fprintf('样本数: %d\n', num_samples);
fprintf('q(s) 是否发生移动: %d\n', q_is_moving);
fprintf('max|es|   = %.4f m\n', max(abs(es)));
fprintf('max|ed|   = %.4f m\n', max(abs(ed)));
fprintf('max|echi| = %.6f rad\n', max(abs(echi)));
fprintf('最终 leg_index = %d\n', round(leg_index(end)));
fprintf('最终 s = %.2f m\n', s(end));

figure('Name', 'AFCS Horizontal Geometry');
hold on

h_wp = plot(flight_plan(1,3), flight_plan(1,2), 'pentagram', ...
    'MarkerSize', 12, 'MarkerFaceColor', 'b');
h_center = [];
h_ref = [];

for i = 2:size(flight_plan, 1)
    h_wp = plot(flight_plan(i,3), flight_plan(i,2), 'pentagram', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'b');

    if flight_plan(i,1) > 1.5
        h_center = plot(flight_plan(i,8), flight_plan(i,7), 'o', ...
            'MarkerSize', 6, 'MarkerFaceColor', 'b');
        plot([flight_plan(i,8), flight_plan(i,3)], ...
             [flight_plan(i,7), flight_plan(i,2)], 'b:', 'LineWidth', 0.2);
        plot([flight_plan(i,8), flight_plan(i-1,3)], ...
             [flight_plan(i,7), flight_plan(i-1,2)], 'b:', 'LineWidth', 0.2);
    end

    if flight_plan(i,1) < 1.5
        point_tf = build_tf_leg_points(flight_plan, i);
        h_ref = plot(point_tf(:,2), point_tf(:,1), 'b', 'LineWidth', 2);
    else
        point_arc = build_rf_arc_points(flight_plan, i);
        h_ref = plot(point_arc(:,2), point_arc(:,1), 'b', 'LineWidth', 2);
    end
end

h_plane = [];
if has_plane_track
    h_plane = plot(plane_lon, plane_lat, 'r', 'LineWidth', 2);
else
    warning('plot_afcs_system_model_results:MissingPlaneTrack', ...
        '未找到飞机经纬度日志，当前只绘制参考路径与 q(s)。');
end

q_plot_step = max(1, floor(num_samples / 250));
h_q = plot(q_lon(1:q_plot_step:end), q_lat(1:q_plot_step:end), 'g^', ...
    'LineStyle', 'none', 'MarkerSize', 5, 'MarkerFaceColor', 'g');

grid on
set(gca, 'FontSize', Font_size, 'FontName', Font_name);
xlabel('Longitude (°)', 'FontSize', Font_size, 'FontName', Font_name)
ylabel('Latitude (°)', 'FontSize', Font_size, 'FontName', Font_name)

legend_handles = h_wp;
legend_text = {'waypoint'};
if ~isempty(h_center) && isgraphics(h_center)
    legend_handles(end+1) = h_center; %#ok<AGROW>
    legend_text{end+1} = 'center of RF leg'; %#ok<AGROW>
end
if ~isempty(h_ref) && isgraphics(h_ref)
    legend_handles(end+1) = h_ref; %#ok<AGROW>
    legend_text{end+1} = 'reference horizontal trajectory'; %#ok<AGROW>
end
if ~isempty(h_plane) && isgraphics(h_plane)
    legend_handles(end+1) = h_plane; %#ok<AGROW>
    legend_text{end+1} = 'actual horizontal trajectory'; %#ok<AGROW>
end
legend_handles(end+1) = h_q;
legend_text{end+1} = 'virtual target q(s)';
legend(legend_handles, legend_text, 'FontSize', Font_size, 'FontName', Font_name);

figure('Name', 'AFCS System Model Monitor', 'Position', [120 100 1800 1200]);
set(gcf, 'Color', 'w');

% ---- 1. 经纬度轨迹图 ----
subplot(3,4,1);
hold on; grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--', 'Color', 'w');
plot_reference_path_only(flight_plan);
if has_plane_track
    h_plane = plot(plane_lon, plane_lat, 'r-', 'LineWidth', 1.8);
else
    h_plane = [];
end
h_q = plot(q_lon(1:q_plot_step:end), q_lat(1:q_plot_step:end), 'g^', ...
    'LineStyle', 'none', 'MarkerSize', 4.5, 'MarkerFaceColor', 'g');
xlabel('经度 (°)', 'FontSize', Font_size, 'FontName', Font_name);
ylabel('纬度 (°)', 'FontSize', Font_size, 'FontName', Font_name);
title('经纬度轨迹图', 'FontSize', Font_size + 1, 'FontName', Font_name);
set(gca, 'FontSize', Font_size, 'FontName', Font_name);

% 手动构建 legend，避免重复条目
legend_handles = [];
legend_text = {};
% 找到第一条蓝色参考线
h_blue = findobj(gca, 'Type', 'line', 'Color', 'b');
if ~isempty(h_blue)
    legend_handles = [legend_handles, h_blue(end)];
    legend_text{end+1} = '参考路径';
end
if has_plane_track && ~isempty(h_plane)
    legend_handles = [legend_handles, h_plane];
    legend_text{end+1} = '飞机轨迹';
end
legend_handles = [legend_handles, h_q];
legend_text{end+1} = '虚拟目标 q(s)';
legend(legend_handles, legend_text, ...
    'FontSize', Font_size, 'FontName', Font_name, 'Box', 'off');

% ---- 2. 纵向误差 es ----
subplot(3,4,2);
hold on; grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--', 'Color', 'w');
plot(time_vec, es, 'b-', 'LineWidth', 1.5, 'DisplayName', 'es (m)');
yline(0, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
xlabel('时间 (s)', 'FontSize', Font_size, 'FontName', Font_name);
ylabel('纵向误差 (m)', 'FontSize', Font_size, 'FontName', Font_name);
title(sprintf('纵向误差 es  (max=%.3f m)', max(abs(es))), ...
    'FontSize', Font_size + 1, 'FontName', Font_name);
set(gca, 'FontSize', Font_size, 'FontName', Font_name);
legend('Location', 'best', 'Box', 'off');

% ---- 3. 横向误差 ed ----
subplot(3,4,3);
hold on; grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--', 'Color', 'w');
plot(time_vec, ed, 'r-', 'LineWidth', 1.5, 'DisplayName', 'ed (m)');
yline(0, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
xlabel('时间 (s)', 'FontSize', Font_size, 'FontName', Font_name);
ylabel('横向误差 (m)', 'FontSize', Font_size, 'FontName', Font_name);
title(sprintf('横向误差 ed  (max=%.3f m)', max(abs(ed))), ...
    'FontSize', Font_size + 1, 'FontName', Font_name);
set(gca, 'FontSize', Font_size, 'FontName', Font_name);
legend('Location', 'best', 'Box', 'off');

% ---- 4. 航向误差 echi ----
subplot(3,4,4);
hold on; grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--', 'Color', 'w');
plot(time_vec, echi, 'g-', 'LineWidth', 1.5, 'DisplayName', 'echi (rad)');
yline(0, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
xlabel('时间 (s)', 'FontSize', Font_size, 'FontName', Font_name);
ylabel('航向误差 (rad)', 'FontSize', Font_size, 'FontName', Font_name);
title(sprintf('航向误差 echi  (max=%.4f°)', max(abs(echi)) * 180/pi), ...
    'FontSize', Font_size + 1, 'FontName', Font_name);
set(gca, 'FontSize', Font_size, 'FontName', Font_name);
legend('Location', 'best', 'Box', 'off');

% ---- 5. 推进速度 s_dot ----
subplot(3,4,5);
hold on; grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--', 'Color', 'w');
plot(time_vec, s_dot, 'm-', 'LineWidth', 1.5, 'DisplayName', 's\_dot (m/s)');
xlabel('时间 (s)', 'FontSize', Font_size, 'FontName', Font_name);
ylabel('速度 (m/s)', 'FontSize', Font_size, 'FontName', Font_name);
title('虚拟目标推进速度', 'FontSize', Font_size + 1, 'FontName', Font_name);
set(gca, 'FontSize', Font_size, 'FontName', Font_name);
legend('Location', 'best', 'Box', 'off');

% ---- 6. 路径坐标 s ----
subplot(3,4,6);
hold on; grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--', 'Color', 'w');
plot(time_vec, s, 'b-', 'LineWidth', 1.5, 'DisplayName', 's (m)');
xlabel('时间 (s)', 'FontSize', Font_size, 'FontName', Font_name);
ylabel('路径坐标 s (m)', 'FontSize', Font_size, 'FontName', Font_name);
title('虚拟目标推进路径', 'FontSize', Font_size + 1, 'FontName', Font_name);
set(gca, 'FontSize', Font_size, 'FontName', Font_name);
legend('Location', 'best', 'Box', 'off');

% ---- 7. 切向角 chi_f ----
subplot(3,4,7);
hold on; grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--', 'Color', 'w');
plot(time_vec, chi_f, 'b-', 'LineWidth', 1.5, 'DisplayName', 'chi\_f (rad)');
xlabel('时间 (s)', 'FontSize', Font_size, 'FontName', Font_name);
ylabel('路径切向角 (rad)', 'FontSize', Font_size, 'FontName', Font_name);
title('切向角', 'FontSize', Font_size + 1, 'FontName', Font_name);
set(gca, 'FontSize', Font_size, 'FontName', Font_name);
legend('Location', 'best', 'Box', 'off');

% ---- 8. 路径曲率 kappa ----
subplot(3,4,8);
hold on; grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--', 'Color', 'w');
plot(time_vec, kappa, 'r-', 'LineWidth', 1.5, 'DisplayName', 'kappa (1/m)');
yline(0, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
xlabel('时间 (s)', 'FontSize', Font_size, 'FontName', Font_name);
ylabel('曲率 (1/m)', 'FontSize', Font_size, 'FontName', Font_name);
title('路径曲率', 'FontSize', Font_size + 1, 'FontName', Font_name);
set(gca, 'FontSize', Font_size, 'FontName', Font_name);
legend('Location', 'best', 'Box', 'off');

% ---- 9. 期望航向角速度 omega_d ----
subplot(3,4,9);
hold on; grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--', 'Color', 'w');
plot(time_vec, omega_d, 'c-', 'LineWidth', 1.5, 'DisplayName', 'omega_d (rad/s)');
yline(0, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
xlabel('时间 (s)', 'FontSize', Font_size, 'FontName', Font_name);
ylabel('期望航向角速度 (rad/s)', 'FontSize', Font_size, 'FontName', Font_name);
title(sprintf('期望航向角速度 omega_d (max=%.4f rad/s)', max(abs(omega_d))), ...
    'FontSize', Font_size + 1, 'FontName', Font_name);
set(gca, 'FontSize', Font_size, 'FontName', Font_name);
legend('Location', 'best', 'Box', 'off');

rad2deg = 180 / pi;

% ---- 10. 滚转指令 phi_c ----
subplot(3,4,10);
hold on; grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--', 'Color', 'w');
plot(time_vec, phi_c * rad2deg, 'm-', 'LineWidth', 1.5, 'DisplayName', 'phi_c (deg)');
yline(0, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
xlabel('时间 (s)', 'FontSize', Font_size, 'FontName', Font_name);
ylabel('滚转指令 (°)', 'FontSize', Font_size, 'FontName', Font_name);
title(sprintf('滚转指令 phi_c (max=%.2f°)', max(abs(phi_c)) * rad2deg), ...
    'FontSize', Font_size + 1, 'FontName', Font_name);
set(gca, 'FontSize', Font_size, 'FontName', Font_name);
legend('Location', 'best', 'Box', 'off');

% ---- 11. 自适应时间常数估计 lambda ----
subplot(3,4,11);
hold on; grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--', 'Color', 'w');
plot(time_vec, lambda_phi_hat, 'Color', [0 0.6 0.8], 'LineWidth', 1.5, 'DisplayName', 'lambda (s)');
xlabel('时间 (s)', 'FontSize', Font_size, 'FontName', Font_name);
ylabel('时间常数估计 (s)', 'FontSize', Font_size, 'FontName', Font_name);
title(sprintf('自适应时间常数 lambda (final=%.4f s)', lambda_phi_hat(end)), ...
    'FontSize', Font_size + 1, 'FontName', Font_name);
set(gca, 'FontSize', Font_size, 'FontName', Font_name);
legend('Location', 'best', 'Box', 'off');

% ---- 12. 航向角速度跟踪误差 omega_e ----
subplot(3,4,12);
hold on; grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--', 'Color', 'w');
plot(time_vec, omega_e, 'k-', 'LineWidth', 1.5, 'DisplayName', 'omega_e (rad/s)');
yline(0, 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
xlabel('时间 (s)', 'FontSize', Font_size, 'FontName', Font_name);
ylabel('航向角速度误差 (rad/s)', 'FontSize', Font_size, 'FontName', Font_name);
title(sprintf('航向角速度跟踪误差 omega_e (max=%.4f rad/s)', max(abs(omega_e))), ...
    'FontSize', Font_size + 1, 'FontName', Font_name);
set(gca, 'FontSize', Font_size, 'FontName', Font_name);
legend('Location', 'best', 'Box', 'off');

end

function [plane_lat, plane_lon, has_plane_track] = get_plane_track()
plane_lat = [];
plane_lon = [];

[plane_lat, found_lat] = try_get_logged_signal({'Plane_Lati', 'Latitude'});
[plane_lon, found_lon] = try_get_logged_signal({'Plane_Longi', 'Longitude'});

has_plane_track = found_lat && found_lon && ~isempty(plane_lat) && ~isempty(plane_lon);
if ~has_plane_track
    plane_lat = [];
    plane_lon = [];
end
end

function [q_lat, q_lon, es, ed, echi, s, leg_index, chi_f, kappa, s_dot, ...
    omega_d, phi_c, lambda_phi_hat, omega_e] = get_monitor_outputs()
if base_var_exists('SM_vector')
    raw = evalin('base', 'SM_vector');
    sm_matrix = convert_monitor_matrix(raw, 14);
else
    error('plot_afcs_system_model_results:MissingMonitorData', ...
        '基工作区缺少 SM_vector，请先仿真并记录系统模型输出。');
end

q_lat = sm_matrix(:,1);
q_lon = sm_matrix(:,2);
es = sm_matrix(:,3);
ed = sm_matrix(:,4);
echi = sm_matrix(:,5);
s = sm_matrix(:,6);
leg_index = sm_matrix(:,7);
chi_f = sm_matrix(:,8);
kappa = sm_matrix(:,9);
s_dot = sm_matrix(:,10);
omega_d = sm_matrix(:,11);
phi_c = sm_matrix(:,12);
lambda_phi_hat = sm_matrix(:,13);
omega_e = sm_matrix(:,14);
end

function time_vec = get_time_vector(num_samples)
if base_var_exists('Sim_time')
    time_vec = convert_logged_value_to_vector(evalin('base', 'Sim_time'));
    return;
end

if base_var_exists('tout')
    time_vec = convert_logged_value_to_vector(evalin('base', 'tout'));
    return;
end

if base_var_exists('SM_vector')
    raw = evalin('base', 'SM_vector');
    time_vec = try_extract_time(raw);
    if ~isempty(time_vec)
        return;
    end
end

[plane_lat, ~, has_plane_track] = get_plane_track();
if has_plane_track && numel(plane_lat) >= num_samples
    time_vec = (0:num_samples-1).';
    warning('plot_afcs_system_model_results:MissingTime', ...
        '未找到时间变量，当前用样本序号代替时间轴。');
    return;
end

time_vec = (0:num_samples-1).';
warning('plot_afcs_system_model_results:MissingTime', ...
    '未找到时间变量，当前用样本序号代替时间轴。');
end

function [value, found] = try_get_logged_signal(name_candidates)
value = [];
found = false;

for i = 1:numel(name_candidates)
    name = name_candidates{i};
    if base_var_exists(name)
        value = convert_logged_value_to_vector(evalin('base', name));
        found = true;
        return;
    end

    dataset_value = try_get_from_dataset('logsout', name);
    if ~isempty(dataset_value)
        value = dataset_value;
        found = true;
        return;
    end

    dataset_value = try_get_from_dataset('yout', name);
    if ~isempty(dataset_value)
        value = dataset_value;
        found = true;
        return;
    end
end
end

function value = try_get_from_dataset(dataset_name, signal_name)
value = [];

if ~base_var_exists(dataset_name)
    return;
end

dataset_value = evalin('base', dataset_name);
if ~isa(dataset_value, 'Simulink.SimulationData.Dataset')
    return;
end

for idx = 1:dataset_value.numElements
    elem = dataset_value.getElement(idx);
    if strcmp(elem.Name, signal_name)
        value = convert_logged_value_to_vector(elem.Values);
        return;
    end
end
end

function time_vec = try_extract_time(raw)
time_vec = [];

if isa(raw, 'timeseries')
    time_vec = raw.Time(:);
    return;
end

if isstruct(raw)
    if isfield(raw, 'time')
        time_vec = raw.time(:);
        return;
    end
    if isfield(raw, 'Time')
        time_vec = raw.Time(:);
        return;
    end
end
end

function value = convert_logged_value_to_vector(raw)
if isnumeric(raw)
    value = squeeze(raw);
elseif isstruct(raw) && isfield(raw, 'signals') && isfield(raw.signals, 'values')
    value = squeeze(raw.signals.values);
elseif isa(raw, 'timeseries')
    value = squeeze(raw.Data);
else
    error('plot_afcs_system_model_results:UnsupportedType', ...
        '当前日志变量类型暂不支持。');
end

value = value(:);
end

function matrix_value = convert_monitor_matrix(raw_vector, output_size)
if isnumeric(raw_vector)
    raw_matrix = squeeze(raw_vector);
elseif isstruct(raw_vector) && isfield(raw_vector, 'signals') && isfield(raw_vector.signals, 'values')
    raw_matrix = squeeze(raw_vector.signals.values);
elseif isa(raw_vector, 'timeseries')
    raw_matrix = squeeze(raw_vector.Data);
else
    error('plot_afcs_system_model_results:UnsupportedSMVector', ...
        'SM_vector 的数据类型暂不支持。');
end

if isvector(raw_matrix)
    raw_matrix = raw_matrix(:);
end

if size(raw_matrix, 2) >= output_size
    matrix_value = raw_matrix(:, 1:output_size);
elseif size(raw_matrix, 1) >= output_size
    matrix_value = raw_matrix(1:output_size, :).';
else
    error('plot_afcs_system_model_results:InvalidSMVector', ...
        'SM_vector 维度不符合预期，无法拆分成 %d 个输出。', output_size);
end
end

function point_tf = build_tf_leg_points(flight_plan, i)
start_lat = flight_plan(i-1, 2);
start_lon = flight_plan(i-1, 3);
end_lat = flight_plan(i, 2);
end_lon = flight_plan(i, 3);

leg_data = func_GreatCircleInverse(start_lat, start_lon, end_lat, end_lon);
leg_length = leg_data(1);
leg_bearing = leg_data(2);

num_points = max(2, ceil(leg_length / 200) + 1);
s_samples = linspace(0, leg_length, num_points);
point_tf = zeros(num_points, 2);

for idx = 1:num_points
    pt = func_GreatCircleForward(start_lat, start_lon, leg_bearing, s_samples(idx));
    point_tf(idx, :) = pt;
end
end

function point_arc = build_rf_arc_points(flight_plan, i)
leg_c2s = func_RhumbLineInverse(flight_plan(i,7), flight_plan(i,8), ...
    flight_plan(i-1,2), flight_plan(i-1,3));
leg_c2e = func_RhumbLineInverse(flight_plan(i,7), flight_plan(i,8), ...
    flight_plan(i,2), flight_plan(i,3));
Q_Angle = func_CalculateArcAngle(leg_c2s(2), leg_c2e(2), flight_plan(i,5));

record = 0;
point_arc = [];

if flight_plan(i,5) > 0
    final_angle = leg_c2s(2) + Q_Angle;
    if final_angle > 360
        for k = leg_c2s(2):0.2:360
            record = record + 1;
            point_arc(record,:) = func_GreatCircleForward(flight_plan(i,7), flight_plan(i,8), k, flight_plan(i,6)); %#ok<AGROW>
        end
        for k = 0:0.2:leg_c2e(2)
            record = record + 1;
            point_arc(record,:) = func_GreatCircleForward(flight_plan(i,7), flight_plan(i,8), k, flight_plan(i,6)); %#ok<AGROW>
        end
    else
        for k = leg_c2s(2):0.2:leg_c2e(2)
            record = record + 1;
            point_arc(record,:) = func_GreatCircleForward(flight_plan(i,7), flight_plan(i,8), k, flight_plan(i,6)); %#ok<AGROW>
        end
    end
else
    final_angle = leg_c2s(2) - Q_Angle;
    if final_angle < 0
        for k = leg_c2e(2):0.2:360
            record = record + 1;
            point_arc(record,:) = func_GreatCircleForward(flight_plan(i,7), flight_plan(i,8), k, flight_plan(i,6)); %#ok<AGROW>
        end
        for k = 0:0.2:leg_c2s(2)
            record = record + 1;
            point_arc(record,:) = func_GreatCircleForward(flight_plan(i,7), flight_plan(i,8), k, flight_plan(i,6)); %#ok<AGROW>
        end
    else
        for k = leg_c2e(2):0.2:leg_c2s(2)
            record = record + 1;
            point_arc(record,:) = func_GreatCircleForward(flight_plan(i,7), flight_plan(i,8), k, flight_plan(i,6)); %#ok<AGROW>
        end
    end
end
end

function plot_reference_path_only(flight_plan)
for i = 2:size(flight_plan,1)
    if flight_plan(i,1) < 1.5
        point_tf = build_tf_leg_points(flight_plan, i);
        plot(point_tf(:,2), point_tf(:,1), ...
             'b', 'LineWidth', 2.2, 'DisplayName', '参考路径');
    else
        point_arc = build_rf_arc_points(flight_plan, i);
        plot(point_arc(:,2), point_arc(:,1), ...
             'b', 'LineWidth', 2.2, 'DisplayName', '参考路径');
    end
end
end

function tf = base_var_exists(var_name)
tf = evalin('base', sprintf('exist(''%s'', ''var'')', var_name)) ~= 0;
end
