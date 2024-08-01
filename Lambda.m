function [L] = Lambda(q)
% Lambda is Jacobian of quaternion derivative (q-dot) w.r.t. angular velocity (w).
%
% Input:
% q - quaternion [4x1].
%
% Return:
% [4x3] matrix.
%
L = 0.5*[-q(2) -q(3) -q(4);
    q(1) -q(4) q(3);
    q(4) q(1) -q(2);
    -q(3) q(2) q(1)];
end