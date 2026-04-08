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