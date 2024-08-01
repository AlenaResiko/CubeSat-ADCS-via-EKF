function [F_out] = F(in_vector,J1,J2,J3,tStep)
% finds gradient of f w.r.t. x
q = in_vector(1:4);
w = in_vector(5:7);
% Omega is 4x4, Lambda is 4x3, 0 is 3x3, Pi is 3x3
F_out = eye(7) + [0.5*Omega(w) 0.5*Lambda(q);
    zeros(3,4) Pi(w,J1,J2,J3)]*tStep;
end