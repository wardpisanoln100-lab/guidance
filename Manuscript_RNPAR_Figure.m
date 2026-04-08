% RNP AR飞行程序绘图
Font_name = 'Times New Roman';
Font_size    = 14;

%% 绘制水平飞行航迹
figure;
% 绘制第一个航路点
plot(RNPAR_FlightPlan(1,3),RNPAR_FlightPlan(1,2),'pentagram','MarkerSize',12,'MarkerFaceColor','b');
hold on
for i = 2:size(RNPAR_FlightPlan,1)
    % 绘制航路点
    h0 = plot(RNPAR_FlightPlan(i,3),RNPAR_FlightPlan(i,2),'pentagram','MarkerSize',12,'MarkerFaceColor','b');
    hold on
    % 绘制RF航段圆心
    if RNPAR_FlightPlan(i,1) > 1.5 %为RF航段
        h1 = plot(RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i,7),'o','MarkerSize',6,'MarkerFaceColor','b');
        hold on
        % 用虚线绘制圆心与圆弧航段起点、终点的连线
        plot([RNPAR_FlightPlan(i,8) RNPAR_FlightPlan(i,3)],[RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,2)],'b:','linewidth',0.2);
        hold on
        plot([RNPAR_FlightPlan(i,8) RNPAR_FlightPlan(i-1,3)],[RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i-1,2)],'b:','linewidth',0.2);
        hold on
    end
    % 绘制参考航迹
    if RNPAR_FlightPlan(i,1) < 1.5 % TF航段
        h2 = plot([RNPAR_FlightPlan(i-1,3) RNPAR_FlightPlan(i,3)],[RNPAR_FlightPlan(i-1,2) RNPAR_FlightPlan(i,2)],'b','linewidth',2);
        hold on
    else  % RF航段
        % RF航段的参考航迹绘制较为麻烦
        % 首先计算从圆心到航段起点的方位角
        leg_c2s = func_RhumbLineInverse(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i-1,2),RNPAR_FlightPlan(i-1,3));
        % 再计算从圆心到航段终点的方位角
        leg_c2e = func_RhumbLineInverse(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i,2),RNPAR_FlightPlan(i,3));
        % 结合转弯方向计算圆弧航段对应的圆心角
        Q_Angle = func_CalculateArcAngle(leg_c2s(2),leg_c2e(2),RNPAR_FlightPlan(i,5));
        % 圆弧右转，即顺时针转弯
        if RNPAR_FlightPlan(i,5) > 0 % 右转
            % 判断是否跨过360/0°线
            Final_Angle = leg_c2s(2) + Q_Angle;
            if Final_Angle > 360 % 跨过360°线，则先绘制从航段起点到360，再绘制从0到终点
                record = 0;
                Point_Arc = [];
                for k = leg_c2s(2):0.2:360 % 0.2°一个步长，可以调整
                    record = record+1;
                    % 解算圆弧上的点，从起点到正北
                    Point_Arc(record,:) = func_GreatCincleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
                for k = 0:0.2:leg_c2e(2)
                    record = record+1;
                    % 解算圆弧上的点，从正北到终点
                    Point_Arc(record,:) = func_GreatCincleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
            else
                record = 0;
                Point_Arc = [];
                for k = leg_c2s(2):0.2:leg_c2e(2)
                    record = record+1;
                    % 解算圆弧上的点，从正北到终点
                    Point_Arc(record,:) = func_GreatCincleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
            end 
        % 圆弧左转，即逆时针转弯
        else  % 左转
            % 判断是否跨过360/0°线
            Final_Angle = leg_c2s(2) - Q_Angle;
            if Final_Angle < 0 %跨过360°线，则反向画图，先绘制从航段终点到360，再绘制从0到航段起点
                record = 0;
                Point_Arc = [];
                for k = leg_c2e(2):0.2:360 % 0.2°一个步长，可以调整
                    record = record+1;
                    % 解算圆弧上的点，从起点到正北
                    Point_Arc(record,:) = func_GreatCincleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
                for k = 0:0.2:leg_c2s(2)
                    record = record+1;
                    % 解算圆弧上的点，从正北到终点
                    Point_Arc(record,:) = func_GreatCincleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
            else
                record = 0;
                Point_Arc = [];
                for k = leg_c2e(2):0.2:leg_c2s(2)
                    record = record+1;
                    % 解算圆弧上的点，从正北到终点
                    Point_Arc(record,:) = func_GreatCincleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end                    
            end
        end
        h2 = plot(Point_Arc(:,2),Point_Arc(:,1),'b','linewidth',2);
        hold on
    end
end
% 实际飞行航迹
h3 = plot(Plane_Longi, Plane_Lati,'r','linewidth',2);
grid on
set(gca,'FontSize',Font_size,'FontName',Font_name);%设置坐标轴刻度字体大小
xlabel('Longitude (°)','FontSize',Font_size,'FontName',Font_name)
ylabel('Latitude (°)','FontSize',Font_size,'FontName',Font_name)
legend([h0 h1 h2 h3],'waypoint','center of RF leg','reference horizontal trajectory','actual horizontal trajectory','FontSize',Font_size,'FontName',Font_name)

%% 绘制垂直飞行航迹
figure;
% 垂直航迹按照航程和高度绘制
% 计算各航段的航段长度
Voyage_Leg = [];
Voyage_Leg(1) = 0; %表示第一个航路点处的飞行距离为0
for i = 2:size(RNPAR_FlightPlan,1)
    if RNPAR_FlightPlan(i,1) < 1.5 %为TF航段
        % 直接利用大圆航线反解求航段长度
        leg_s2e = func_RhumbLineInverse(RNPAR_FlightPlan(i-1,2), RNPAR_FlightPlan(i-1,3),RNPAR_FlightPlan(i,2),RNPAR_FlightPlan(i,3));
        Voyage_Leg(i) = leg_s2e(1) + Voyage_Leg(i-1);
    else % 为RF航段
        % 计算圆弧航段对应的圆心角
        % 首先计算从圆心到航段起点的方位角
        leg_c2s = func_RhumbLineInverse(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i-1,2),RNPAR_FlightPlan(i-1,3));
        % 再计算从圆心到航段终点的方位角
        leg_c2e = func_RhumbLineInverse(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i,2),RNPAR_FlightPlan(i,3));
        % 结合转弯方向计算圆弧航段对应的圆心角
        Q_Angle = func_CalculateArcAngle(leg_c2s(2),leg_c2e(2),RNPAR_FlightPlan(i,5));
        % 根据圆的周长计算公式计算圆弧航段的长度
        Voyage_Leg(i) = Q_Angle*2*pi*RNPAR_FlightPlan(i,6)/360 + Voyage_Leg(i-1);
    end
end
% 绘图
ReferenceAlt = [];
for i = 1:size(RNPAR_FlightPlan,1)
    ReferenceAlt(i) = RNPAR_FlightPlan(i,4);
    % 航路点
    h0 = plot(Voyage_Leg(i),RNPAR_FlightPlan(i,4),'pentagram','MarkerSize',12,'MarkerFaceColor','b');
    hold on
end
% 参考航迹
h1 = plot(Voyage_Leg,ReferenceAlt,'b','linewidth',2);
hold on
% 实际飞行航迹
h2 = plot(FlightDistance, Plane_Height,'r','linewidth',2);
grid on
set(gca,'FontSize',Font_size,'FontName',Font_name);%设置坐标轴刻度字体大小
xlabel('Voyage (m)','FontSize',Font_size,'FontName',Font_name)
ylabel('Altitude (m)','FontSize',Font_size,'FontName',Font_name)
legend([h0 h1 h2],'waypoint','reference vertical trajectory','actual vertical trajectory','FontSize',Font_size,'FontName',Font_name)
    