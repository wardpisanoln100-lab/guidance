function r = drl_piecewise_reward(x)
% drl_piecewise_reward - 论文公式 9 的分段奖励函数 r(x)
%
% 参数: c1=-1, c2=1, c3=1, c4=-1
%
%           { -x^2,              |x| < 1
% r1(x) =   {
%           { -x^2 * exp(-x^2),   |x| >= 1
%
% r2(x) = exp(-x^2)
% r(x)  = r1(x) + r2(x)

c1 = -1;
c2 = 1;
c3 = 1;
c4 = -1;

ax = abs(x);

if ax <= c2
    r1 = c1 * x^2;
    r2 = c3 * exp(c4 * x^2);
else
    r1 = c1 * c3 * c4 * x^2 * exp(c4 * x^2);
    r2 = c3 * exp(c4 * x^2);
end

r = r1 + r2;
end
