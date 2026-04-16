function [delta_ed, delta_echi] = func_DRLCompensationBlock(beta, V_y, p, r, ed, es, echi, kappa, phi, drl_enable)
% func_DRLCompensationBlock - Simulink MATLAB Function Block Entry Point
%
% DRL-based error compensation wrapper for Y-8 aircraft lateral control.
% Intended to be used inside a Simulink "MATLAB Function" block.
%
% Input Ports (10):
%   1. beta        : Sideslip angle (rad)
%   2. V_y         : Lateral velocity (m/s)
%   3. p           : Roll rate (rad/s)
%   4. r           : Yaw rate (rad/s)
%   5. ed          : S-F lateral error (m)
%   6. es          : S-F longitudinal position (m)     % NEW
%   7. echi        : Heading error (rad)
%   8. kappa       : Path curvature (1/m)              % NEW
%   9. phi         : Roll angle (rad)
%   10. drl_enable : Switch (1=enabled, 0=disabled)
%
% Output Ports (2):
%   1. delta_ed    : DRL-generated ed compensation (m)
%   2. delta_echi  : DRL-generated echi compensation (rad)
%
% Notes:
%   - When drl_enable == 0, outputs are [0, 0].
%   - Requires drl_actor_weights.mat in the working directory when enabled.
%   - Internally calls drl_error_compensation(obs) with a 9-D observation vector.

delta_ed = 0;
delta_echi = 0;

if drl_enable
    obs = [beta; V_y; p; r; ed; es; echi; kappa; phi];
    [delta_ed, delta_echi] = drl_error_compensation(obs);
end
end
