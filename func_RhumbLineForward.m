function out = func_RhumbLineForward( Latitude0,Longitude0,Bearing,length_leg)
%等角航线正解计算公式
%即已知航段起点经纬度坐标、航段长度和航向，求航段终点的经纬度坐标
%输入参数
%Latitude0， 起点的纬度，单位deg
%Longitude0，起点的经度，单位deg
%length_leg, 等角航线的长度，单位，米
%Bearing，   航段方位角，单位deg
%输出参数
%Latitude， 终点的纬度，单位deg
%Longitude，终点的经度，单位deg
%计算所用常数
%地球半径，单位米
 R_earth=6378137;
 %WGS-84模型的偏心率
 e=0.0818191908426215;%1/298.257;


%单位转换 
C_deg2rad=180/pi;
if Bearing>89.9&&Bearing<90.1
    Bearing=90;
else if Bearing>269.9&&Bearing<270.1
        Bearing=270;
    end
end
Latitude0=Latitude0/C_deg2rad;
Longitude0=Longitude0/C_deg2rad;
Bearing=Bearing/C_deg2rad;

if Bearing==pi/2
    Latitude=Latitude0;
    Longitude=length_leg*(1-e^2*(sin(Latitude0))^2)^0.5/(R_earth*cos(Latitude0))+Longitude0;
elseif Bearing==3*pi/2
        Latitude=Latitude0;
    Longitude=-length_leg*(1-e^2*(sin(Latitude0))^2)^0.5/(R_earth*cos(Latitude0))+Longitude0;
else
    k1=1+3*e^2/4+45*e^4/64+175*e^6/256+11025*e^8/16384;
    k2=-3*e^2/8-15*e^4/32-525*e^6/1024-2205*e^8/4096;
    k3=15*e^4/256+105*e^6/1024+2205*e^8/16384;
    k4=-35*e^6/3072-105*e^8/4096;
    k5=315*e^8/131072;
    xl1=R_earth*(1-e^2)*(k1*Latitude0+k2*sin(2*Latitude0)+k3*sin(4*Latitude0)+k4*sin(6*Latitude0)+k5*sin(8*Latitude0));
    xl2=xl1+length_leg*cos(Bearing);
    a2=3*e^2/8+3*e^4/16+213*e^6/2048+255*e^8/4096;
    a4=21*e^4/256+21*e^6/256+533*e^8/8192;
    a6=151*e^6/6144+151*e^8/4096;
    a8=1097*e^8/131082;
    phi=xl2/(R_earth*(1-e^2)*(1+3*e^2/4+45*e^4/64+175*e^6/256+11025*e^8/16384));
    Latitude=phi+a2*sin(2*phi)+a4*sin(4*phi)+a6*sin(6*phi)+a8*sin(8*phi);
    q1=atanh(sin(Latitude0))-e*atanh(e*sin(Latitude0));
    q2=atanh(sin(Latitude))-e*atanh(e*sin(Latitude));
    Longitude=Longitude0+(q2-q1)*tan(Bearing);   
end

    Latitude=real(Latitude*C_deg2rad);
    Longitude=real(Longitude*C_deg2rad);
    %函数输出
    out(1) = Latitude;
    out(2) = Longitude;
end