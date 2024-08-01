function [Pi_out] = Pi(w,J1,J2,J3)
% Pi is Jacobian of angular acceleration (omega-dot) w.r.t. angular velocity (omega).
%
% Input:
% w - angular velocity vector [3x1].
% J1, J2, J3 - moments of inertia along principal axes.
%
% Return:
% [3x3] matrix.
%
Pi_out = [0 (J2-J3)/J1*w(3) (J2-J3)/J1*w(2);
    (J3-J1)/J2*w(3) 0 (J3-J1)/J2*w(1);
    (J1-J2)/J3*w(2) (J1-J2)/J3*w(1) 0];
end