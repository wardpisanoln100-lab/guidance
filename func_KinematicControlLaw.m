function [omega_d, s_dot, detail] = func_KinematicControlLaw(es, ed, echi, V_chi, kappa, params)
% func_KinematicControlLaw - 复现运动学控制律（Eq.5、Eq.6a、Eq.6b）
%
% 输入:
%   es, ed, echi : S-F 误差状态
%   V_chi        : 水平速度（m/s）
%   kappa        : 路径曲率（1/m）
%   params       : 必填参数结构体（统一在 system_model_init.m 设置），字段包括
%                  ks, k_omega, gamma, k, chi_inf
%
% 输出:
%   omega_d : 期望航向角速度指令（rad/s）
%   s_dot   : 弧长变化率（m/s）
%   detail  : 中间量，便于调试与核对

if nargin < 6 || isempty(params)
    error('func_KinematicControlLaw:MissingParams', ...
        ['缺少 params。请在 system_model_init.m 中设置参数，并通过 ' ...
         'func_SystemModelMonitorBlock 传入。']);
end

ks = get_required_param(params, 'ks');
k_omega = get_required_param(params, 'k_omega');
gamma = get_required_param(params, 'gamma');
k = get_required_param(params, 'k');
chi_inf = get_required_param(params, 'chi_inf');

% Eq.5: 进场角 delta(ed)
% delta = -chi_inf * (exp(2*k*ed)-1)/(exp(2*k*ed)+1)
% 为避免 exp 溢出，对指数自变量进行限幅
z = 2 .* k .* ed;
z = min(max(z, -60), 60);
exp_z = exp(z);
delta = -chi_inf .* (exp_z - 1) ./ (exp_z + 1);

% d(delta)/d(ed)
delta_prime = -chi_inf .* (4 .* k .* exp_z) ./ ((exp_z + 1) .^ 2);

% Eq.6b: 弧长动态
s_dot = ks .* es + V_chi .* cos(echi);

% Eq.6a: 期望航向角速度指令
d = echi - delta;
sinc_term = sinc_half(d);

omega_d = -k_omega .* (echi - delta) + kappa .* s_dot ...
    + delta_prime .* (V_chi .* sin(echi) - kappa .* es .* s_dot) ...
    - (ed .* V_chi ./ gamma) .* sinc_term .* cos((echi + delta) ./ 2);

if nargout >= 3
    detail.delta = delta;
    detail.delta_prime = delta_prime;
    detail.sinc_half = sinc_term;
    detail.ks = ks;
    detail.k_omega = k_omega;
    detail.gamma = gamma;
    detail.k = k;
    detail.chi_inf = chi_inf;
end
end

function v = get_required_param(params, name)
if ~isfield(params, name) || isempty(params.(name))
    error('func_KinematicControlLaw:MissingParamField', ...
        'params 缺少字段 "%s"。请在 system_model_init.m 中设置该参数。', name);
end
v = params.(name);
end

function y = sinc_half(x)
% sinc_half(x) = sin(x/2)/(x/2)，接近零时使用泰勒展开避免数值问题
y = ones(size(x));
idx = abs(x) > 1e-8;
if any(idx(:))
    y(idx) = sin(x(idx) ./ 2) ./ (x(idx) ./ 2);
end

% 零附近二阶泰勒近似
if any(~idx(:))
    z = x(~idx);
    y(~idx) = 1 - (z.^2) ./ 24;
end
end
