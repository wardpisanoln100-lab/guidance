%% 大圆航线正解函数
%输入：航段起点纬度、经度，航段方位角、航段长度
%输出：航段终点纬度、经度
function out=func_GreatCincleForward(StartLati,StartLongi,Bearing,LegLength)
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