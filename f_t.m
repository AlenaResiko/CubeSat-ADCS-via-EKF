function out_vector = f_t(in_vector, w_dot)
    %F_T Returns state based on previous state; using physical motion laws.
    %   takes in previous state
    %   returns next state

    % integrating q_dot and w
    q = in_vector(1:4);
    w = in_vector(5:7);
    dt = Delta_t/10;
    for t = 0:dt:Delta_t
        q = q + 0.5*(Lambda(q)*w)*dt;
        w = w + w_dot*dt; % here w_dot is assumed to be constant for 0:Delta_t period
    out_vector = CAT(1,q,w);
end