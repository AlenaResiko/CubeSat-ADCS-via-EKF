function [P_out] = Pi(w,J1,J2,J3)
% J1, J2, J3 are constants
P_out = [0 (J2-J3)/J1*w(3) (J2-J3)/J1*w(2);
    (J3-J1)/J2*w(3) 0 (J3-J1)/J2*w(1);
    (J1-J2)/J3*w(2) (J1-J2)/J3*w(1) 0];
end