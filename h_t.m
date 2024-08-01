function out_vector = h_t(in_vector, B_o)
    out_vector = zeros(6,1); % set up dimension
    out_vector(1:3) = transformationMatrix(in_vector(1:4)) * B_o; % read from pre-calculated predicted_orbit_file predicted measurement.
    out_vector(4:6) = in_vector(5:7);
end


% check that out_vector(1:3) is normalize SANE)

% normalize at each step and look at angles NOT.SANE

% reduce noise of measurements -> reduced theta SANE