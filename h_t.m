function out_vector = h_t(in_vector, B_o)
% h_t is function mapping state to predicted magnetic field measurement in
% NED frame.
% Returns [6x1] vector.
%
% Input:
% in_vector - state vector [7x1].
% B_o - observed measurement from Magnetic Field block in NED frame.

    out_vector = zeros(6,1); % set up dimension
    % convert predicted magnetic field measurement this from NED to Body 
    % frame using transposed transformation matrix.
    out_vector(1:3) = transformationMatrix(in_vector(1:4)) * B_o;
    out_vector(4:6) = in_vector(5:7); % copy predicted angular velocity
end