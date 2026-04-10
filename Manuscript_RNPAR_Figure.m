% RNP AR���г����ͼ
Font_name = 'Times New Roman';
Font_size    = 14;

%% ����ˮƽ���к���
figure;
% ���Ƶ�һ����·��
plot(RNPAR_FlightPlan(1,3),RNPAR_FlightPlan(1,2),'pentagram','MarkerSize',12,'MarkerFaceColor','b');
hold on
for i = 2:size(RNPAR_FlightPlan,1)
    % ���ƺ�·��
    h0 = plot(RNPAR_FlightPlan(i,3),RNPAR_FlightPlan(i,2),'pentagram','MarkerSize',12,'MarkerFaceColor','b');
    hold on
    % ����RF����Բ��
    if RNPAR_FlightPlan(i,1) > 1.5 %ΪRF����
        h1 = plot(RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i,7),'o','MarkerSize',6,'MarkerFaceColor','b');
        hold on
        % �����߻���Բ����Բ��������㡢�յ������
        plot([RNPAR_FlightPlan(i,8) RNPAR_FlightPlan(i,3)],[RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,2)],'b:','linewidth',0.2);
        hold on
        plot([RNPAR_FlightPlan(i,8) RNPAR_FlightPlan(i-1,3)],[RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i-1,2)],'b:','linewidth',0.2);
        hold on
    end
    % ���Ʋο�����
    if RNPAR_FlightPlan(i,1) < 1.5 % TF����
        h2 = plot([RNPAR_FlightPlan(i-1,3) RNPAR_FlightPlan(i,3)],[RNPAR_FlightPlan(i-1,2) RNPAR_FlightPlan(i,2)],'b','linewidth',2);
        hold on
    else  % RF����
        % RF���εĲο��������ƽ�Ϊ�鷳
        % ���ȼ����Բ�ĵ��������ķ�λ��
        leg_c2s = func_RhumbLineInverse(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i-1,2),RNPAR_FlightPlan(i-1,3));
        % �ټ����Բ�ĵ������յ�ķ�λ��
        leg_c2e = func_RhumbLineInverse(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i,2),RNPAR_FlightPlan(i,3));
        % ���ת�䷽�����Բ�����ζ�Ӧ��Բ�Ľ�
        Q_Angle = func_CalculateArcAngle(leg_c2s(2),leg_c2e(2),RNPAR_FlightPlan(i,5));
        % Բ����ת����˳ʱ��ת��
        if RNPAR_FlightPlan(i,5) > 0 % ��ת
            % �ж��Ƿ���360/0����
            Final_Angle = leg_c2s(2) + Q_Angle;
            if Final_Angle > 360 % ���360���ߣ����Ȼ��ƴӺ�����㵽360���ٻ��ƴ�0���յ�
                record = 0;
                Point_Arc = [];
                for k = leg_c2s(2):0.2:360 % 0.2��һ�����������Ե���
                    record = record+1;
                    % ����Բ���ϵĵ㣬����㵽����
                    Point_Arc(record,:) = func_GreatCircleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
                for k = 0:0.2:leg_c2e(2)
                    record = record+1;
                    % ����Բ���ϵĵ㣬���������յ�
                    Point_Arc(record,:) = func_GreatCircleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
            else
                record = 0;
                Point_Arc = [];
                for k = leg_c2s(2):0.2:leg_c2e(2)
                    record = record+1;
                    % ����Բ���ϵĵ㣬���������յ�
                    Point_Arc(record,:) = func_GreatCircleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
            end 
        % Բ����ת������ʱ��ת��
        else  % ��ת
            % �ж��Ƿ���360/0����
            Final_Angle = leg_c2s(2) - Q_Angle;
            if Final_Angle < 0 %���360���ߣ�����ͼ���Ȼ��ƴӺ����յ㵽360���ٻ��ƴ�0���������
                record = 0;
                Point_Arc = [];
                for k = leg_c2e(2):0.2:360 % 0.2��һ�����������Ե���
                    record = record+1;
                    % ����Բ���ϵĵ㣬����㵽����
                    Point_Arc(record,:) = func_GreatCircleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
                for k = 0:0.2:leg_c2s(2)
                    record = record+1;
                    % ����Բ���ϵĵ㣬���������յ�
                    Point_Arc(record,:) = func_GreatCircleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end
            else
                record = 0;
                Point_Arc = [];
                for k = leg_c2e(2):0.2:leg_c2s(2)
                    record = record+1;
                    % ����Բ���ϵĵ㣬���������յ�
                    Point_Arc(record,:) = func_GreatCircleForward(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),k,RNPAR_FlightPlan(i,6));
                end                    
            end
        end
        h2 = plot(Point_Arc(:,2),Point_Arc(:,1),'b','linewidth',2);
        hold on
    end
end
% ʵ�ʷ��к���
h3 = plot(Plane_Longi, Plane_Lati,'r','linewidth',2);
grid on
set(gca,'FontSize',Font_size,'FontName',Font_name);%����������̶������С
xlabel('Longitude (��)','FontSize',Font_size,'FontName',Font_name)
ylabel('Latitude (��)','FontSize',Font_size,'FontName',Font_name)
legend([h0 h1 h2 h3],'waypoint','center of RF leg','reference horizontal trajectory','actual horizontal trajectory','FontSize',Font_size,'FontName',Font_name)

%% ���ƴ�ֱ���к���
figure;
% ��ֱ�������պ��̺͸߶Ȼ���
% ��������εĺ��γ���
Voyage_Leg = [];
Voyage_Leg(1) = 0; %��ʾ��һ����·�㴦�ķ��о���Ϊ0
for i = 2:size(RNPAR_FlightPlan,1)
    if RNPAR_FlightPlan(i,1) < 1.5 %ΪTF����
        % ֱ�����ô�Բ���߷����󺽶γ���
        leg_s2e = func_RhumbLineInverse(RNPAR_FlightPlan(i-1,2), RNPAR_FlightPlan(i-1,3),RNPAR_FlightPlan(i,2),RNPAR_FlightPlan(i,3));
        Voyage_Leg(i) = leg_s2e(1) + Voyage_Leg(i-1);
    else % ΪRF����
        % ����Բ�����ζ�Ӧ��Բ�Ľ�
        % ���ȼ����Բ�ĵ��������ķ�λ��
        leg_c2s = func_RhumbLineInverse(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i-1,2),RNPAR_FlightPlan(i-1,3));
        % �ټ����Բ�ĵ������յ�ķ�λ��
        leg_c2e = func_RhumbLineInverse(RNPAR_FlightPlan(i,7), RNPAR_FlightPlan(i,8),RNPAR_FlightPlan(i,2),RNPAR_FlightPlan(i,3));
        % ���ת�䷽�����Բ�����ζ�Ӧ��Բ�Ľ�
        Q_Angle = func_CalculateArcAngle(leg_c2s(2),leg_c2e(2),RNPAR_FlightPlan(i,5));
        % ����Բ���ܳ����㹫ʽ����Բ�����εĳ���
        Voyage_Leg(i) = Q_Angle*2*pi*RNPAR_FlightPlan(i,6)/360 + Voyage_Leg(i-1);
    end
end
% ��ͼ
ReferenceAlt = [];
for i = 1:size(RNPAR_FlightPlan,1)
    ReferenceAlt(i) = RNPAR_FlightPlan(i,4);
    % ��·��
    h0 = plot(Voyage_Leg(i),RNPAR_FlightPlan(i,4),'pentagram','MarkerSize',12,'MarkerFaceColor','b');
    hold on
end
% �ο�����
h1 = plot(Voyage_Leg,ReferenceAlt,'b','linewidth',2);
hold on
% ʵ�ʷ��к���
h2 = plot(FlightDistance, Plane_Height,'r','linewidth',2);
grid on
set(gca,'FontSize',Font_size,'FontName',Font_name);%����������̶������С
xlabel('Voyage (m)','FontSize',Font_size,'FontName',Font_name)
ylabel('Altitude (m)','FontSize',Font_size,'FontName',Font_name)
legend([h0 h1 h2],'waypoint','reference vertical trajectory','actual vertical trajectory','FontSize',Font_size,'FontName',Font_name)
    