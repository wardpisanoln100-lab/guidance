function out = func_ApproachGuidance_RNPAR( uu )
% 此函数用于解算RNP AR进近引导过程中的引导参数和引导指令
% 作者: Shaobo
% 时间: 2024-01-18
% 输入: 
% ① RNPAR飞行计划信息，定义格式参见Manuscript_RNPAR_FlightPlan.m
% ②飞机状态参数,纬度、经度、高度、地速；
% ③飞机上一时刻位置,纬度和经度，用于航段切换
% ④当前航段序号，用于更新航段序号
% ⑤仿真时间，只是用来判断起始
% 输出: 
% 当前航段信息(16)，航段总数, 航段序号, 是否最后一个航段(0, 否; 1, 是), 航段类型(0, TF; 1, RF), 
%                             航段起点纬经高、航段终点纬经高、航段方位角、下一航段方位角
%                             圆弧航段圆心经纬度、圆弧航段转弯方向、圆弧航段转弯半径
% 飞行引导指令(5)，侧偏距、航迹方位角、滚转角、高度、垂直速度

% RNP AR进近程序的第一个点一定是IF，或者说不论哪个飞行计划都是如此
Leg_Index  = 0;   % 航段序号
switch_flag = 0; % 航段切换标志


% 输入拆解
number_plan_Info = length(uu) - 8;
u_plan        = uu(1: number_plan_Info);
u_aircraft    = uu((number_plan_Info+1): (number_plan_Info+4));
u_lastpos    = uu((number_plan_Info+5): (number_plan_Info+6));
u_legindex  = uu(number_plan_Info+7);
u_time        = uu(number_plan_Info+8);

% 注意这里输入的u_plan是重新排列之后的，即u_plan = reshape(FlightPlan,1,size(FlightPlan,1)*size(FlightPlan,2));
% 飞行计划的重新排列在函数外完成，即在Simulink模块中完成

% 飞行计划解析，得到可飞航段信息
Flight_Leg = func_FlightPlanAnalysis(u_plan);

% 航段序号更新
if u_time < 0.1
    Leg_Index = 1;
else
    Leg_Index = u_legindex;
end

% 获取当前航段信息
Cur_Leg = Flight_Leg(Leg_Index,:);

% 航段切换判断
if Leg_Index < Flight_Leg(1,1)
    switch_flag = func_HalfPlane([u_lastpos(1), u_lastpos(2), u_aircraft(1), u_aircraft(2), Cur_Leg(8), Cur_Leg(9), Cur_Leg(11), Cur_Leg(12)]);
    if switch_flag > 0.5
        Leg_Index = Leg_Index + 1; % 切换，航段序号+1
    end
else
    Leg_Index = Flight_Leg(1,1);
end

% 更新当前航段信息
Cur_Leg = Flight_Leg(Leg_Index,:);

% 计算飞行引导指令
Guidance_Para = func_GuidanceParaCalculation(u_aircraft, Cur_Leg);

% 结果输出
out(1:16)   = Cur_Leg; %当前航段信息
out(17:21) = Guidance_Para; % 引导参数和引导指令信息
end

%% 飞行计划解析函数
function out = func_FlightPlanAnalysis(uu)
% 此函数用于飞行计划解析
% 输入：RNP AR飞行计划信息（9）
% 飞行计划定义为：
%  航路点类型(0, IF; 1, TF; 2, RF), 航路点纬度、航路点经度、航路点高度、
%  航路点转弯方向(-1, 逆时针L；1, 顺时针R)、航路点转弯半径、圆心纬度、圆心经度、RNP要求
% 输出：解析得到的所有航段信息
% 每条航段信息(16)，航段总数, 航段序号, 是否最后一个航段(0, 否; 1, 是), 航段类型(0, TF; 1, RF), 
%                             航段起点纬经高、航段终点纬经高、航段方位角、下一航段方位角
%                             圆弧航段圆心纬经度、圆弧航段转弯方向、圆弧航段转弯半径

Index = 0;   %航段序号
M = 9;         % 一条飞行计划的长度    
N  = 16;       %解析后每条航段的列数，即航段信息
LegInverseData = [0 0];        % 当前航段反解参数

% 获取飞行航路点总数
Total_waypoint = length(uu)/M;
% 首先进行飞行计划的重新排列
uu_FlightPlan = reshape(uu,Total_waypoint,M); %此时的飞行计划已经为K*9,即K行9列
% 则总的航段数应当为航路点数-1
Leg = zeros(Total_waypoint-1,N);
% 进行航段解析,因为第一个航路点为IF，所以，直接从第二个航路点开始解析
for  i = 2:Total_waypoint
    if uu_FlightPlan(i,1) < 1.5 % 表示当前航段为TF航段
        Index = Index + 1;
        Leg(Index,1) = Total_waypoint-1; % 航段总数
        Leg(Index,2) = Index; % 当前航段序号
        if i < Total_waypoint
            Leg(Index,3) = 0; % 不是结束航段
        else
            Leg(Index,3) = 1; % 是最后一个结束航段
        end
        
        Leg(Index,4) = 0; %TF航段
        
        Leg(Index,5) = uu_FlightPlan(i-1,2);
        Leg(Index,6) = uu_FlightPlan(i-1,3);
        Leg(Index,7) = uu_FlightPlan(i-1,4); % 航段起点为上一个航路点的纬经高
        
        Leg(Index,8) = uu_FlightPlan(i,2);
        Leg(Index,9) = uu_FlightPlan(i,3);
        Leg(Index,10) = uu_FlightPlan(i,4);    % 航段终点为当前航路点的纬经高     
        
        % 计算当前航段方位角，下一航段方位角最后统一处理
        LegInverseData = func_GreatCircleInverse(Leg(Index,5),Leg(Index,6),Leg(Index,8),Leg(Index,9));
        Leg(Index,11) = LegInverseData(2);
        
        % 直线航段，和圆弧航段相关的信息赋值为0
        Leg(Index,13) = 0;
        Leg(Index,14) = 0; % 圆心纬度、经度
        Leg(Index,15) = 0; % 转弯方向
        Leg(Index,16) = 0; % 转弯半径
    else %表示当前航段为RF航段
        Index = Index + 1;
        Leg(Index,1) = Total_waypoint-1; % 航段总数
        Leg(Index,2) = Index; % 当前航段序号
        if i < Total_waypoint
            Leg(Index,3) = 0; % 不是结束航段
        else
            Leg(Index,3) = 1; % 是最后一个结束航段
        end
        
        Leg(Index,4) = 1; % RF航段
        
        Leg(Index,5) = uu_FlightPlan(i-1,2);
        Leg(Index,6) = uu_FlightPlan(i-1,3);
        Leg(Index,7) = uu_FlightPlan(i-1,4); % 航段起点为上一个航路点的纬经高
        
        Leg(Index,8) = uu_FlightPlan(i,2);
        Leg(Index,9) = uu_FlightPlan(i,3);
        Leg(Index,10) = uu_FlightPlan(i,4);    % 航段终点为当前航路点的纬经高     
        
        % 计算当前航段方位角，下一航段方位角最后统一处理
        LegInverseData = func_GreatCircleInverse(Leg(Index,5),Leg(Index,6),Leg(Index,8),Leg(Index,9));
        Leg(Index,11) = LegInverseData(2);
        
        % 圆弧航段相关信息
        Leg(Index,13) = uu_FlightPlan(i,7);
        Leg(Index,14) = uu_FlightPlan(i,8); % 圆心纬度、经度
        Leg(Index,15) = uu_FlightPlan(i,5); % 转弯方向
        Leg(Index,16) = uu_FlightPlan(i,6); % 转弯半径
    end
end%结束整个飞行计划解析过程
% 计算下一航段航段方位角
for i = 1:Index-1
    Leg(i, 12) = Leg(i+1,11);
end
Leg(Index,12) = Leg(Index,11);%最后一个航段的航段方位角不发生变化
% 结果输出，保持原本的矩阵输出
out = Leg;
end

%% 转弯方向判断函数，用于进行航段切换判断
function out = func_JudgeTurnDire( chi_i,chi_f )
% 判断圆弧航段的转弯方向
% 输入：前一航段的转弯方向，后一航段的转弯方向
% 输出：转弯方向，右转为1，左转为-1
delta_chi = chi_f-chi_i;
%  将方位角偏差转换到0-360
if delta_chi < 0
    delta_chi = delta_chi + 360;
end
% 判断转弯方向
if (delta_chi > 0)&&(delta_chi < 180)
    out = 1; %右转（顺时针）
else
    out = -1; %左转（逆时针）
end
end

%% 等角航线下的侧偏距计算函数
function out = func_CalXTDrhumb(Plane_Lati,Plane_Longi,Leg_Lati,Leg_Longi,psi_rh)
%  输入：飞机瞬时位置（纬度、经度），航段起点（纬度、经度），psi_rh（Deg）航线航向；
%  输出：等角航线侧偏距D 
%  说明：规定侧偏距的符号是左正右负，这样，当飞机左偏时，应该右滚转，能够保证侧偏距前边的符号是正数

P   = [Plane_Lati, Plane_Longi];% 飞机位置
P0 = [Leg_Lati,Leg_Longi];%航段起点
C   = 180/pi;
R_L = 6371393;
EPS = 0.00000001;
% 单位转换
L = P(1)/C;
lambda = P(2)/C;
L0 = P0(1)/C;
lambda0 = P0(2)/C;
psi_rh = psi_rh/C;

if abs(psi_rh-pi/2)<EPS 
    out = (L-L0)*R_L;
elseif abs(psi_rh-3/2*pi)<EPS
    % 规定侧偏距左正右负
    % 与航段航向为90°的情况正好相反
    % 航段航向为270°，飞机纬度大于航段起点纬度，位于航段右侧，侧偏距为负
    % 航段航向为270°，飞机纬度小于航段起点纬度，位于航段左侧，侧偏距为正
    out = -(L-L0)*R_L;
else
    lambda1 = lambda0 + tan(psi_rh)*log( (cos(L0)*(1+sin(L)))/(cos(L)*(1+sin(L0))) );
    out(1) = -(lambda-lambda1)*R_L*cos(L)*cos(psi_rh);
end 
end

%% 过半平面切换判断
function out = func_HalfPlane( uu )
% 此函数用于航段切换判断
% 输入：上一时刻飞机位置、当前时刻飞机位置、飞机过点位置(当前航段终点)、当前航段方位角、下一航段方位角
% 输出：航段切换标志位，0表示不进行航段切换，1表示进行航段切换
% 输入获取
% 定义极小值
EPS0 = 1e-8;
% 上一时刻飞机位置
Last_lati        = uu(1);
Last_longi     = uu(2);
% 当前时刻飞机位置
Cur_lati         = uu(3);
Cur_longi      = uu(4);
% 航路点位置
Cross_lati      = uu(5);
Cross_longi   = uu(6);
% 当前航段方位角
Bearing_s  = uu(7);
% 下一航段方位角
Bearing_e = uu(8);

% 判断航段切换时的转弯方向
turn_direction =  func_JudgeTurnDire(Bearing_s,Bearing_e);
% 单位转换
Bearing_s  = Bearing_s*pi/180;
Bearing_e = Bearing_e*pi/180;
% 计算前后两个航段的夹角
Delta_Angle = Bearing_s-Bearing_e;
if abs(Delta_Angle) < pi
    Angle_in_Leg = pi - abs(Delta_Angle);
end
if abs(Delta_Angle) > pi
    Angle_in_Leg = abs(Delta_Angle) - pi;
end
% 计算切换平面与地平面的交线的方位角
DAngle = Angle_in_Leg/2;
if (Bearing_s > (0-EPS0))&&(Bearing_s <= DAngle)
    if turn_direction < 0
        Chi_HalfPlane = Bearing_s+DAngle;
    end
    if turn_direction > 0
        Chi_HalfPlane = Bearing_s-DAngle+2*pi;
    end
end
if (Bearing_s > DAngle)&&(Bearing_s <= (2*pi-DAngle))
    if turn_direction < 0
        Chi_HalfPlane = Bearing_s+DAngle;
    end
    if turn_direction > 0
        Chi_HalfPlane = Bearing_s-DAngle;
    end
end
if (Bearing_s > (2*pi-DAngle))&&(Bearing_s < (2*pi+EPS0))
    if turn_direction < 0
        Chi_HalfPlane = Bearing_s+DAngle-2*pi;
    end
    if turn_direction > 0
        Chi_HalfPlane = Bearing_s-DAngle;
    end
end
% 归一化处理，0-360
Chi_HalfPlane = Chi_HalfPlane*180/pi;
if (Chi_HalfPlane >= 360)
    Chi_HalfPlane = Chi_HalfPlane-360;
end
if (Chi_HalfPlane < 0)
    Chi_HalfPlane = Chi_HalfPlane+360;
end
% 计算飞机相对于切换平面与地平面的交线的侧偏距
% 这里因为距离很短，所以采用的是等角航线下的侧偏距计算方法
XTK_Last = func_CalXTDrhumb(Last_lati,Last_longi,Cross_lati,Cross_longi,Chi_HalfPlane);
XTK_Cur  = func_CalXTDrhumb(Cur_lati,Cur_longi,Cross_lati,Cross_longi,Chi_HalfPlane);
% 切换判断
if XTK_Last*XTK_Cur <= 0
    Switch_Flag = 1;
else
    Switch_Flag = 0;
end
% 输出
out = Switch_Flag;
end

%% 大圆航线下的侧偏距计算公式
function out = func_CalXTDGreatCircle(uu)
% 输入：飞机当前位置(纬经高), 当前航段起点(纬经), 当前航段终点(纬经)
% 输出: 飞机相对于TF航段的侧偏距，左正右负

% 参数定义
C = 180/pi;
R_L = 6371393;

% 赋值
P   = uu(1:2); % 飞机当前位置，纬经
h   = uu(3);   %   当前飞行高度
P1 = uu(4:5); %  TF航段起点
P2 = uu(6:7); %  TF航段终点

L = P(1)/C;
lambda = P(2)/C; 

L1 = P1(1)/C;
lambda1 = P1(2)/C;
L2 = P2(1)/C;
lambda2 = P2(2)/C;

u = [cos(L)*cos(lambda) cos(L)*sin(lambda) sin(L)];
u_P1 = [cos(L1)*cos(lambda1) cos(L1)*sin(lambda1) sin(L1)];
u_P2 = [cos(L2)*cos(lambda2) cos(L2)*sin(lambda2) sin(L2)];

temp = cross(u_P1,u_P2);
u_P1P2 = temp/norm(temp);
D = (R_L+h)*asin(dot(u,u_P1P2));

% 结果输出
out = D;
end
%% 大圆航线反解函数
%输入：航段起点纬度、经度，航段终点纬度、经度
%输出：航段长度、航段方位角（度）
function out=func_GreatCircleInverse(StartLati,StartLongi,EndLati,EndLongi)
StartLati=StartLati/57.3;
StartLongi=StartLongi/57.3;
EndLati=EndLati/57.3;
EndLongi=EndLongi/57.3;
A = sin((EndLati - StartLati) / 2)*sin((EndLati - StartLati) / 2) + cos(StartLati)*cos(EndLati)*sin((EndLongi - StartLongi) / 2)*sin((EndLongi - StartLongi) / 2);	
RNG = 2 * atan(sqrt(A / (1 - A)));
LegLength = RNG*6378140;
Bearing = atan2(cos(EndLati)*sin(EndLongi - StartLongi) , (cos(StartLati)*sin(EndLati) - cos(EndLati)*sin(StartLati)*cos(EndLongi - StartLongi)));
if Bearing<0
    Bearing=Bearing+2*pi;
end
Bearing=Bearing*180/pi;
%结果输出
out(1) = LegLength;
out(2) = Bearing;
end

%% 大圆航线正解函数
%输入：航段起点纬度、经度，航段方位角、航段长度
%输出：航段终点纬度、经度
function out=func_GreatCircleForward(StartLati,StartLongi,Bearing,LegLength)
    RNG = LegLength / 6378140;
	Bearing = Bearing / (180/pi);
	StartLati = StartLati / (180/pi);
	StartLongi = StartLongi / (180/pi);
    Lati=asin(sin(StartLati)*cos(RNG) + cos(StartLati)*sin(RNG)*cos(Bearing));
    Longi=StartLongi + atan(sin(RNG)*sin(Bearing) / (cos(StartLati)*cos(RNG) - sin(StartLati)*sin(RNG)*cos(Bearing)));
    Lati=Lati*180/pi;
    Longi=Longi*180/pi;
    %结果输出
    out(1) = Lati;
    out(2) = Longi;
end

%% 圆弧航段圆心角计算函数
function out = func_CalculateArcAngle(angle_c2s, angle_c2e, turn_dir)
% 输入: 圆心到圆弧航段起点的航向角、圆心到圆弧航段终点的航向角、圆弧航段的转弯方向
% 输出: 圆弧航段对应圆心角的大小，deg

if turn_dir < 0 % 左转
    if angle_c2e < angle_c2s
        Q = angle_c2s - angle_c2e;
    else
        Q = 360 + angle_c2s - angle_c2e;
    end
else % 右转
    if angle_c2e > angle_c2s
        Q = angle_c2e - angle_c2s;
    else
        Q = 360 + angle_c2e - angle_c2s;
    end
end

if (Q >= 359.5) || (Q < 0.5)
    Q = 0;
end
% 结果输出
out = Q;
end

%% 引导参数计算和引导指令生成函数
function out = func_GuidanceParaCalculation(u_aircraft, u_leg)
% 输入：飞机状态参数(纬经高、地速)、当前航段信息
% 每条航段信息(16)，航段总数, 航段序号, 是否最后一个航段(0, 否; 1, 是), 航段类型(0, TF; 1, RF), 
%                             航段起点纬经高、航段终点纬经高、航段方位角、下一航段方位角
%                             圆弧航段圆心纬经度、圆弧航段转弯方向、圆弧航段转弯半径
% 输出：引导参数和指令→侧偏距、航迹方位角、滚转角、高度、垂直速度(XTK, chi_g, phi_g, H_g, Vs_g)

persistent XTK_filt

% 参数初始化
Para_XTK  = 0;
Para_chi   = 0;
Para_phi   = 0;
Para_H     = 0;
Para_VS    = 0;

% 引导参数和引导计算
if u_leg(4) < 0.5 %TF航段
    % 计算侧偏距
    Para_XTK = func_CalXTDGreatCircle([u_aircraft(1), u_aircraft(2), u_aircraft(3), u_leg(5), u_leg(6), u_leg(8), u_leg(9)]);
    % 计算航段方位角
    Para_chi  = u_leg(11);
    % 计算滚转角
    Para_phi  = 0; %直线航段期望滚转角为0
    % 飞机当前位置到航段起点的距离
    leg_p2s = func_GreatCircleInverse(u_aircraft(1), u_aircraft(2), u_leg(5), u_leg(6));
    % 当前航段的长度
    leg_s2e = func_GreatCircleInverse(u_leg(5), u_leg(6), u_leg(8), u_leg(9));
    % 计算航段梯度
    k_slope = (u_leg(10) - u_leg(7))/abs(leg_s2e(1));
    % 计算期望高度
    Para_H = u_leg(7) + k_slope*abs(leg_p2s(1));
    % 计算目标垂直速度
    Para_VS = u_aircraft(4)*k_slope;
else % RF航段
    % 计算飞机到圆心的距离
    leg_p2c = func_GreatCircleInverse(u_aircraft(1), u_aircraft(2), u_leg(13), u_leg(14));
    Dis_XTD = abs(leg_p2c(1));
    % 计算侧偏距
    % 侧偏距规定左正右负，右转弯时，Dis_XTD > Leg_R，侧偏距为正，Dis_XTD < Leg_R，侧偏距为负
    Para_XTK = u_leg(15)*(Dis_XTD - u_leg(16));
    % 计算航段方位角
    Para_chi  = u_leg(11);
    % 计算实时航迹方位角
    leg_c2p = func_GreatCircleInverse(u_leg(13), u_leg(14), u_aircraft(1), u_aircraft(2));
    Para_chi = leg_c2p(2) + u_leg(15)*90;
    % 归一化到[0,360)
    if Para_chi >= 360
        Para_chi = Para_chi - 360;
    end
    if Para_chi < 0
        Para_chi = Para_chi + 360;
    end
    % 计算滚转角
    Para_phi = u_leg(15) * atan(u_aircraft(4)*u_aircraft(4)/(9.8*u_leg(16)));
    % Para_phi = u_aircraft(4)*u_aircraft(4)/(9.8*u_leg(16));
    % 计算从圆心到圆弧航段起点的方位角
    leg_c2s = func_GreatCircleInverse(u_leg(13), u_leg(14), u_leg(5), u_leg(6));
    % 计算从圆心到圆弧航段终点的方位角
    leg_c2e = func_GreatCircleInverse(u_leg(13), u_leg(14), u_leg(8), u_leg(9));
    % 计算从圆心到飞机当前位置的方位角
    leg_c2p = func_GreatCircleInverse(u_leg(13), u_leg(14), u_aircraft(1), u_aircraft(2));
    % 计算整个圆弧航段对应的圆心角
    Angle_Qt =  func_CalculateArcAngle(leg_c2s(2), leg_c2e(2), u_leg(15));
    % 计算飞机已经飞过的圆心角
    Angle_Qp =  func_CalculateArcAngle(leg_c2s(2), leg_c2p(2), u_leg(15));                                                                                                                                                                                              
    % 计算期望高度
    Para_H = u_leg(7) + (u_leg(10)- u_leg(7))*Angle_Qp/Angle_Qt;
    % 计算期望垂直速度
    % 这里参照直线航段，航段梯度等于高度差/航段长度
    Para_VS = u_aircraft(4)*(u_leg(10)- u_leg(7))/(Angle_Qt*2*pi*u_leg(16)/360);
end
% 对航迹方位角进行处理--不然0和360没法区分
if (Para_chi<= 360)  && (Para_chi > 359.5)
    Para_chi = 0;
end

% 对侧偏距进行同步淡化，抑制RF/TF切换时的突变
tau_xtk = 0.8; % s, 可在0.5~1.0之间调参
dt = 0.01; % s, 当前函数调用周期
alpha_xtk = dt/(tau_xtk + dt);

if ~isfinite(Para_XTK)
    if isempty(XTK_filt) || ~isfinite(XTK_filt)
        Para_XTK = 0;
    else
        Para_XTK = XTK_filt;
    end
end

if isempty(XTK_filt) || ~isfinite(XTK_filt)
    XTK_filt = Para_XTK;
else
    XTK_filt = alpha_xtk*Para_XTK + (1-alpha_xtk)*XTK_filt;
end

% 结果输出
out(1) = XTK_filt;
out(2) = Para_chi;
out(3) = Para_phi;
out(4) = Para_H;
out(5) = Para_VS;
end






