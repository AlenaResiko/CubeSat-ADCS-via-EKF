function [Om] = Omega(w)
% Omega is Jacobian of quaternion derivative (q-dot) w.r.t. quaternion (q).
%
% Input:
% w - angular velocity vector [3x1].
%
% Return:
% [4x4] matrix.
%
Om = 0.5 * [0, -w(1), -w(2), -w(3); 
     w(1), 0, w(3), -w(2);
     w(2), -w(3), 0, w(1);
     w(3), w(2), -w(1), 0];
end