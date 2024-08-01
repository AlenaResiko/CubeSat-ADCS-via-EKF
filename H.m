function [H_out] = H(in_vector,B_o)
% H is linearization of h_t function (which maps state to predict measurement).
% H is Jacobian of h(x) w.r.t. x.
%
% Input:
% in_vector - state vector [7x1].
% B_o - observed measurement from Magnetic Field block in NED frame.
%
% Returns:
% [6x7] Jaconbian matrix.

q = in_vector(1:4);
w = in_vector(5:7);
% left hand is partial_h/partial_q
dA_T_dq1 = 2 * [ q(1),  q(4), -q(3); 
                -q(4),  q(1),  q(2); 
                 q(3), -q(2),  q(1)];

dA_T_dq2 = 2 * [ q(2),  q(3),  q(4); 
                 q(3), -q(2),  q(1); 
                 q(4), -q(1), -q(2)];

dA_T_dq3 = 2 * [-q(3),  q(2), -q(1); 
                 q(2),  q(3),  q(4); 
                 q(1),  q(4), -q(3)];

dA_T_dq4 = 2 * [-q(4),  q(1),  q(2); 
                -q(1), -q(4),  q(3); 
                 q(2),  q(3),  q(4)];

H_out = [dA_T_dq1 * B_o, dA_T_dq2 * B_o, dA_T_dq3 *B_o, dA_T_dq4 *B_o, zeros(3,3);
         zeros(3,4), eye(3)];
end