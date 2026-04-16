% RNP AR进近程序飞行轨迹图
Font_name = 'Times New Roman';
Font_size    = 14;

%% 绘制水平飞行轨迹
figure;
% 绘制第一个航路点
plot(RNPAR_FlightPlan(1,3),RNPAR_FlightPlan(1,2),'pentagram','MarkerSize',12,'MarkerFaceColor','b');
hold on
for i = 2:size(RNPAR_FlightPlan,1)
    % 绘制航路段
    h0 = plot(RNPAR_FlightPlan(i,3),RNPAR_FlightPlan(i,2),'pentagram','MarkerSize',12,'MarkerFaceColor','b');
    hold on
    % 绘制RF段的圆形
    if RNPAR_FlightPlan(i,1) > 1.5 % 为RF段
        h1 = plot(RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i,7),'o','MarkerSize',6,'MarkerFaceColor','b');
        hold on
        % 辅助线或圆弧线，圆心点、起始点和终点连线
        plot([RNPAR_FlightPlan(i,8) RNPAR_FlightPlan(i,3)],[RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,2)],'b:','linewidth',0.2);
        hold on
        plot([RNPAR_FlightPlan(i,8) RNPAR_FlightPlan(i-1,3)],[RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i-1,2)],'b:','linewidth',0.2);
        hold on
    end
    % 绘制参考轨迹
    if RNPAR_FlightPlan(i,1) < 1.5 % TF段
        h2 = plot([RNPAR_FlightPlan(i-1,3) RNPAR_FlightPlan(i,3)],[RNPAR_FlightPlan(i-1,2) RNPAR_FlightPlan(i,2)],'b','linewidth',2);
        hold on
    else  % RF段
        % RF段的参考轨迹绘制较为麻烦
        % 首先计算圆心点到起始点的方位角
        leg_c2s = func_RhumbLineInverse(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i-1,2),RNPAR_FlightPlan(i-1,3));
        % 再计算圆心点到终点的方位角
        leg_c2e = func_RhumbLineInverse(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i,2),RNPAR_FlightPlan(i,3));
        % 根据转弯方向计算圆弧段对应的圆心角
        Q_Angle = func_CalculateArcAngle(leg_c2s(2),leg_c2e(2),RNPAR_FlightPlan(i,5));
        % 圆弧为顺时针方向旋转
        if RNPAR_FlightPlan(i,5) > 0 % 右转
            % 判断是否跨越360/0度线
            Final_Angle = leg_c2s(2) + Q_Angle;
            if Final_Angle > 360 % 超过360度线，先绘制从起始点到360度，再绘制从0度到终点
                record = 0;
                Point_Arc = [];
                for k = leg_c2s(2):0.2:360 % 0.2度为一个点，步长可以调整
                    record = record+1;
                    % 计算圆弧上的点，从起始点到终点
                    Point_Arc(record,:) = func_GreatCircleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
                for k = 0:0.2:leg_c2e(2)
                    record = record+1;
                    % 计算圆弧上的点，从起点到终点
                    Point_Arc(record,:) = func_GreatCircleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
            else
                record = 0;
                Point_Arc = [];
                for k = leg_c2s(2):0.2:leg_c2e(2)
                    record = record+1;
                    % 计算圆弧上的点，从起点到终点
                    Point_Arc(record,:) = func_GreatCircleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
            end
        % 圆弧为逆时针方向旋转
        else  % 左转
            % 判断是否跨越360/0度线
            Final_Angle = leg_c2s(2) - Q_Angle;
            if Final_Angle < 0 % 跨越360度线，逆时针方向先绘制从起始点到0度，再绘制从360度到终点
                record = 0;
                Point_Arc = [];
                for k = leg_c2e(2):0.2:360 % 0.2度为一个点，步长可以调整
                    record = record+1;
                    % 计算圆弧上的点，从终点到起始点
                    Point_Arc(record,:) = func_GreatCircleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
                for k = 0:0.2:leg_c2s(2)
                    record = record+1;
                    % 计算圆弧上的点，从起点到终点
                    Point_Arc(record,:) = func_GreatCircleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
            else
                record = 0;
                Point_Arc = [];
                for k = leg_c2e(2):0.2:leg_c2s(2)
                    record = record+1;
                    % 计算圆弧上的点，从起点到终点
                    Point_Arc(record,:) = func_GreatCircleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
            end
        end
        h2 = plot(Point_Arc(:,2),Point_Arc(:,1),'b','linewidth',2);
        hold on
    end
end
% 实际飞行轨迹
h3 = plot(Plane_Longi, Plane_Lati,'r','linewidth',2);
grid on
set(gca,'FontSize',Font_size,'FontName',Font_name);% 设置坐标轴字体大小
xlabel('Longitude (°)','FontSize',Font_size,'FontName',Font_name)
ylabel('Latitude (°)','FontSize',Font_size,'FontName',Font_name)
legend([h0 h1 h2 h3],'waypoint','center of RF leg','reference horizontal trajectory','actual horizontal trajectory','FontSize',Font_size,'FontName',Font_name)

%% 绘制垂直飞行轨迹
figure;
% 垂直方向距离和高程计算
% 计算各段的航程距离
Voyage_Leg = [];
Voyage_Leg(1) = 0; % 表示第一个航路点处的飞行距离为0
for i = 2:size(RNPAR_FlightPlan,1)
    if RNPAR_FlightPlan(i,1) < 1.5 % 为TF段
        % 直接使用恒向线方法计算航段距离
        leg_s2e = func_RhumbLineInverse(RNPAR_FlightPlan(i-1,2), RNPAR_FlightPlan(i-1,3),RNPAR_FlightPlan(i,2),RNPAR_FlightPlan(i,3));
        Voyage_Leg(i) = leg_s2e(1) + Voyage_Leg(i-1);
    else % 为RF段
        % 计算圆弧段对应的圆心角
        % 首先计算圆心点到起始点的方位角
        leg_c2s = func_RhumbLineInverse(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i-1,2),RNPAR_FlightPlan(i-1,3));
        % 再计算圆心点到终点的方位角
        leg_c2e = func_RhumbLineInverse(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i,2),RNPAR_FlightPlan(i,3));
        % 根据转弯方向计算圆弧段对应的圆心角
        Q_Angle = func_CalculateArcAngle(leg_c2s(2),leg_c2e(2),RNPAR_FlightPlan(i,5));
        % 根据圆周长公式计算圆弧段的长度
        Voyage_Leg(i) = Q_Angle*2*pi*RNPAR_FlightPlan(i,6)/360 + Voyage_Leg(i-1);
    end
end
% 绘图
ReferenceAlt = [];
for i = 1:size(RNPAR_FlightPlan,1)
    ReferenceAlt(i) = RNPAR_FlightPlan(i,4);
    % 各航路点
    h0 = plot(Voyage_Leg(i),RNPAR_FlightPlan(i,4),'pentagram','MarkerSize',12,'MarkerFaceColor','b');
    hold on
end
% 参考轨迹
h1 = plot(Voyage_Leg,ReferenceAlt,'b','linewidth',2);
hold on
% 实际飞行轨迹
h2 = plot(FlightDistance, Plane_Height,'r','linewidth',2);
grid on
set(gca,'FontSize',Font_size,'FontName',Font_name);% 设置坐标轴字体大小
xlabel('Voyage (m)','FontSize',Font_size,'FontName',Font_name)
ylabel('Altitude (m)','FontSize',Font_size,'FontName',Font_name)
legend([h0 h1 h2],'waypoint','reference vertical trajectory','actual vertical trajectory','FontSize',Font_size,'FontName',Font_name)
