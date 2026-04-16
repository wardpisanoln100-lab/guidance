%本程序用来进行螺旋桨飞机的配平和仿真
%clear
warning off 
%clc
AirCraftData;
%飞行包线


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%定义元胞数组，存储飞机配平的方程,
%第一列为重量，第二列为重心位置；
%第三列存储高度值，第四列为速度值；
%第五列为纵向方程A阵，第六列为纵向方程B阵；
%第七列为横航向方程A阵，第八列为横航向方程B阵
% Data_trim=cell(length(H_envolope),4,5);
flap=0;

%飞机重量及重心位置
MASS=[65000 61000 58000 54000 51000];
%disp('请选择飞机重量序号：');
%disp('1  65吨；2  61吨；3  58吨；4  54吨；5  51吨；');
% index_M=input('index_M=');
% mass=MASS(index_M);
% disp('请输入飞机飞行高度 (m)');
% alt=input('alt=');
% disp('请输入飞机飞行速度 (m/s)');
% Vt=input('Vt=');

% disp('请输入襟翼偏度 (度)');
% flap=input('flap=');
Index_Data=0;
%飞机平均气动弦长
CA=3.451;

switch mass
 case 65000
        index_M=1;
        X=1.071; %正常重心位置纵轴坐标
        Y=1.333;
%         disp('please input the value of X, [ 0.824<X<1.175 ]');
%         X=input('X=');
        
        sim('Inertia_Calculate_65');
        
    case 61000
        X=1.067; %重心位置纵轴坐标 
        Y=1.293; %重心位置横轴坐标
%         disp('please input the value of X, [ 0.821<X<1.173 ]');
%         X=input('X=');
%         disp('please input the value of Y, [ 1.223<Y<1.32 ]');
%         Y=input('Y=');
        sim('Inertia_Calculate_61');
        index_M=2;
       
        
    case 58000
        X=1.065; %重心位置纵轴坐标
        Y=1.261;
%         disp('please input the value of X, [ 0.818<X<1.17 ]');
%         X=input('X=');
%         disp('please input the value of Y, [ 1.191<Y<1.285 ]');
%         Y=input('Y=');
        
        sim('Inertia_Calculate_58');
        index_M=3;
        
    case 54000
        X=1.062; %重心位置纵轴坐标
        Y=1.214;
        
%         disp('please input the value of X, [ 0.745<X<1.167 ]');
%         X=input('X=');
%         disp('please input the value of Y, [ 1.126<Y<1.232 ]');
%         Y=input('Y=');
        sim('Inertia_Calculate_54');
        index_M=4;
        
    case 49000
        X=1.06; %重心位置纵轴坐标
        Y=1.177;
        
%          disp('please input the value of X, [ 0.743<X<1.163 ]');
%         X=input('X=');
%         disp('please input the value of Y, [ 1.093<Y<1.182 ]');
%         Y=input('Y=');
        sim('Inertia_Calculate_49');
      
        index_M=5;
        
      
end


    
%%%%%*********赋初值**************



%飞机，在地面坐标系中的位置初值XYZ

xme_0=[0 0 -alt];
Vab_0=[Vt 0 0];%飞行速度(m/s)，迎角和侧滑角（rad）的初值
eul_0=[0/57.3 0/57.3 0/57.3];  %三个欧拉角Phi Theta Psi的初值（rad）
Wxyz_0=[0 0 0]; %飞机转动角速度在飞机机体坐标系三轴的投影，(rad/s)
inertia=[];     %飞机绕机体系三轴的转动惯量和惯性积

Ix=Ix(1);
Iy=Iy(1);
Iz=Iz(1);
Ixz=Ixz(1);   

inertia=[  Ix      Iy     Iz  Ixz];

Inertia_M=[  Ix      0  -Ixz;
               0    Iy    0
           -Ixz      0  Iz];


%%%%%***********************
%
% trimfun;
%%%%%***********************

 opspec = operspec('Aero_Dynamics');
% Aero_Dynamics
% opspec = operspec('Aero_Dynamics');

%%%%************** 输入***************
%副翼偏度，单位rad
opspec.Inputs(1).Known = 0;
opspec.Inputs(1).max = 24/57.3;
opspec.Inputs(1).min = -24/57.3;
opspec.Inputs(1).u = -0/57.3;

%升降舵偏度，单位rad
opspec.Inputs(2).Known = 0;
opspec.Inputs(2).max = 21/57.3;
opspec.Inputs(2).min = -21/57.3;
opspec.Inputs(2).u = 2.5/57.3;

%方向舵偏度，单位rad
opspec.Inputs(3).Known = 0;
opspec.Inputs(3).max = 25/57.3;
opspec.Inputs(3).min = -25/57.3;
opspec.Inputs(3).u = 0/57.3;

%油门输入偏度
opspec.Inputs(4).Known = 0;
opspec.Inputs(4).max = 1.04;
opspec.Inputs(4).min = 0;
opspec.Inputs(4).u =0.4;


% %襟翼偏度，单位rad
% opspec.Inputs(1).Known = 0;
% opspec.Inputs(1).max = 0/57.3;
% opspec.Inputs(1).min = 0/57.3;
% opspec.Inputs(1).u = 0;

%%%%************** 定速直线平飞的设置状态 ***************
% U V W
opspec.States(1).Known =[1;0;0];
opspec.States(1).SteadyState = [1;1;1];
%p q r
opspec.States(2).Known = [1;1;1];
opspec.States(2).SteadyState = [1;1;1];
% euler
opspec.States(3).Known = [0;0;0];
opspec.States(3).SteadyState = [1;1;1];

% x y z
opspec.States(4).Known = [0;0;1];
opspec.States(4).SteadyState =[0;1;1];
% %%%%************** 稳态盘旋设置 ***************
% % U V W
% opspec.States(1).Known =[1;0;0];
% opspec.States(1).SteadyState = [1;1;1];
% %p q r
% opspec.States(2).Known = [1;1;0];
% opspec.States(2).SteadyState = [1;1;1];
% % euler
% opspec.States(3).Known = [0;0;0];
% opspec.States(3).SteadyState = [1;1;0];
% 
% % x y z
% opspec.States(4).Known = [0;0;1];
% opspec.States(4).SteadyState =[0;0;1];

% %%%%************** 稳态升降设置 ***************
% % U V W
% opspec.States(1).Known =[0;0;0];
% opspec.States(1).SteadyState = [1;1;1];
% %p q r
% opspec.States(2).Known = [1;1;1];
% opspec.States(2).SteadyState = [1;1;1];
% % euler
% opspec.States(3).Known = [0;1;1];
% opspec.States(3).SteadyState = [1;1;1];
% 
% % x y z
% opspec.States(4).Known = [0;0;0];
% opspec.States(4).SteadyState =[0;1;0];

%%%%************** 配平 ***************
opt = linoptions('DisplayReport','on');
[op,oprp] = findop('Aero_Dynamics',opspec,opt);
disp(oprp.TerminationString);



%% 将配平状态赋值给线性化模型
set(op,'Model', 'Aero_Dynamics');
update(op);

% %% 线性化
% % 设置线性化输入输出
% modelname = 'Aero_Dynamics_021';
% i=1;
% io(i) = linio([modelname,'/delta_a'],1,'in');i=i+1; 
% io(i) = linio([modelname,'/delta_e'],1,'in');i=i+1;      
% io(i) = linio([modelname,'/delta_r'],1,'in');i=i+1;   
% io(i) = linio([modelname,'/delta_p'],1,'in');i=i+1;   
% 
% %V Alpha Beta
% io(i) = linio([modelname,'/U1'],1,'out');i=i+1; 
% io(i) = linio([modelname,'/U1'],2,'out');i=i+1;  
% io(i) = linio([modelname,'/U1'],3,'out');i=i+1;  
% 
% %p q r
% io(i) = linio([modelname,'/U1'],4,'out');i=i+1;   
% io(i) = linio([modelname,'/U1'],5,'out');i=i+1;  
% io(i) = linio([modelname,'/U1'],6,'out');i=i+1;
% 
% %phi theta psi
% io(i) = linio([modelname,'/U1'],7,'out');i=i+1; 
% io(i) = linio([modelname,'/U1'],8,'out');i=i+1; 
% io(i) = linio([modelname,'/U1'],9,'out');i=i+1;
% 
% %Xe Ye Ze
% io(i) = linio([modelname,'/U1'],10,'out');i=i+1;
% io(i) = linio([modelname,'/U1'],11,'out');i=i+1;
% io(i) = linio([modelname,'/U1'],12,'out'); i=i+1;
% 
% clear i;
% 
% % 在配平点处线性化模型
% sys = linearize(modelname,op,io);
% 
% A = sys.a; B = sys.b; C = sys.c; D = sys.d;
% 
% 
% % %纵向方程,状态变量 V Alpha q theta
% % disp('纵向方程')
% % A_long = sel(A,[1 2 5 8],[1 2  5 8])
% % B_long=sel(B,[1 2 5 8],[2 4])
% % C_long=sel(C,[1 2  5 8],[1 2  5 8])
% % D_long=sel(D,[1 2  5 8],[2 4])
% % 
% % 
% % %横向方程,状态变量 Beta p r phi
% % disp('横航向方程')
% % A_lat=sel(A,[3 4 6 7],[3 4 6 7])
% % B_lat=sel(B,[3 4 6 7],[1 3])
% % C_lat=sel(C,[3 4 6 7],[3 4 6 7])
% % D_lat=sel(D,[3 4 6 7],[1 3])
% % 
% % [Wn_longh,Z_long,P_long]=damp(A_long)
% % [Wn_lat,Z_lat,P_lat]=damp(A_lat)