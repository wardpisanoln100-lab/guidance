function out = func_CalculateArcAngle(angle_c2s, angle_c2e, turn_dir)
% 圆弧航段圆心角计算函数
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