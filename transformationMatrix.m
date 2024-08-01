function [A] = transformationMatrix(q)
% Returns matrix that transforms vector `from Body to NED frame.
% Return [3x3] orthogonal matrix.
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
A = [ q1^2 + q2^2 - q3^2 - q4^2,    2*(q2*q3 - q1*q4),         2*(q2*q4 + q1*q3);
      2*(q2*q3 + q1*q4),            q1^2 - q2^2 + q3^2 - q4^2, 2*(q3*q4 - q1*q2);
      2*(q2*q4 - q1*q3),            2*(q3*q4 + q1*q2),         q1^2 - q2^2 - q3^2 + q4^2];
end