function out = func_RhumbLineInverse(Latitude1,Longitude1,Latitude2,Longitude2 )
%等角航线反解函数
%即已知航段起点经纬度坐标、航段终点的经纬度坐标，求航段长度和航向
%输入参数
%Latitude1  起点的纬度，单位deg
%Longitude1 起点的经度，单位deg
%Latitude2终点的纬度，单位deg
%Longitude2终点的经度，单位deg
%输出参数
%length_leg, 等角航线的长度，单位，米
%Bearing，   航段方位角，单位deg
%地球半径
 R_earth=6378137;
 %WGS-84地球变率
 e=0.0818191908426215;
%单位转换
C_deg2rad=180/pi;
Latitude1=Latitude1/C_deg2rad;
Longitude1=Longitude1/C_deg2rad;
Latitude2=Latitude2/C_deg2rad;
Longitude2=Longitude2/C_deg2rad;
%参数计算
q1=atanh(sin(Latitude1))-e*atanh(e*sin(Latitude1));
q2=atanh(sin(Latitude2))-e*atanh(e*sin(Latitude2));
if Latitude2==Latitude1
    if Longitude2>Longitude1
        Bearing=pi/2;
    else
        Bearing=3*pi/2;
    end
elseif Latitude2>Latitude1
        if Longitude2>Longitude1
            Bearing=atan((Longitude2-Longitude1)/(q2-q1));
        else Bearing=atan((Longitude2-Longitude1)/(q2-q1))+2*pi;
        end
else Bearing=atan((Longitude2-Longitude1)/(q2-q1))+pi;
end
if Bearing==pi/2||Bearing==3*pi/2;
    length_leg=abs(R_earth*cos(Latitude1)*(Longitude2-Longitude1)/(1-e^2*(sin(Latitude1))^2)^0.5);
else
k1=1+3*e^2/4+45*e^4/64+175*e^6/256+11025*e^8/16384;
k2=-3*e^2/8-15*e^4/32-525*e^6/1024-2205*e^8/4096;
k3=15*e^4/256+105*e^6/1024+2205*e^8/16384;
k4=-35*e^6/3072-105*e^8/4096;
k5=315*e^8/131072;
xl1=R_earth*(1-e^2)*(k1*Latitude1+k2*sin(2*Latitude1)+k3*sin(4*Latitude1)+k4*sin(6*Latitude1)+k5*sin(8*Latitude1));
xl2=R_earth*(1-e^2)*(k1*Latitude2+k2*sin(2*Latitude2)+k3*sin(4*Latitude2)+k4*sin(6*Latitude2)+k5*sin(8*Latitude2));

length_leg=abs((xl2-xl1)/cos(Bearing));

end
Bearing=Bearing*C_deg2rad;   
%函数输出
out(1) = length_leg;
out(2) = Bearing;
end

