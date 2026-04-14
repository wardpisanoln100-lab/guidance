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
%
% 特性:
%   x = 0:     r = 1.0    (最大奖励)
%   x = 0.5:   r ≈ 0.529
%   x = 1.0:   r ≈ -0.632 (转折点，连续)
%   x → ∞:     r → 0⁻     (趋近于 0，从负侧)

ax = abs(x);

if ax <= 1
    r1 = -x^2;
    r2 = exp(-x^2);
else
    r1 = -x^2 * exp(-x^2);
    r2 = exp(-x^2);
end

r = r1 + r2;
end
