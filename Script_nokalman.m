clear all; %clear all variables from previous runs

%save simulation outputs to produce results
num_iterations = 100; % number of iteration of the simulation to run
x_list = zeros(7,num_iterations); % time series of the estimated state vector 
q_rotate_list = zeros(4,num_iterations); % time series of the quaternion offset from target
torq_list = zeros(3,num_iterations); % time series of torque implemented
theta_err_list = zeros(1,num_iterations);  %time series of the angle estimation error (estimation theta - true theta)
w_err_list = zeros(1,num_iterations); % time series of the norm of the angular rate estimation error (norm of (estimation w - true w))
w_body2inertial_list = zeros(3,num_iterations);% time series of the angular rate estimation of body frame with respect to inertial frame
counter = 1;%index for logging the timeseries

%getting the model workspace for modifying variables and simulation
mdl = "No_Kalman_FullSystem";
simIN = Simulink.SimulationInput(mdl);
pathSpace = mdl+'/Orbit sim'+'/Spacecraft Dynamics';
pathSpace_mea = mdl+ '/NOKalmanMeasurement_sim' + '/Orbit sim measurement' + '/Spacecraft Dynamics';
mdlWks = get_param('No_Kalman_FullSystem','ModelWorkspace');



%set the time to run one iteration
stepSize = 0.05;
set_param(mdl,'StopTime',mat2str(stepSize))
set_param(mdl,'SolverType','Fixed-step');
set_param(mdl,'FixedStep', mat2str(stepSize))
assignin(mdlWks,'t_step',(stepSize));

%initialize the very initial parameters for theoratical model
jd =2458850;
inertial_x=[3649700.0, 3308200.0, -4676600.0];
inertial_v= [-2750.8, 6666.4, 2573.4];
w = [0.02;0.02;-0.01];
q = [0.2; -0.6 ;-0.5; -0.59160797831];
torq = [0;0;0];

%initialize the very initial parameters for measurement simulation
jd_mea =2458850 ;
inertial_x_mea=[3649700.0, 3308200.0, -4676600.0];
inertial_v_mea= [-2750.8, 6666.4, 2573.4];
w_mea = [0.02;0.02;-0.01];
q_mea = [0.2;-0.6;-0.5;-0.59160797831];

q_ff = [0.76813986749607;-0.00060774689776115;0.00073833382166474;-0.64028136742373]; %initial attitude with respect to fix frame

%noise covariances
R = 0.5*eye(7);
assignin(mdlWks,'NoiseCov',R)

m=100; % mass of CubeSat
assignin(mdlWks,'m',m);

%call the simulation for the assigned number of iterations
for i=(1:num_iterations) 

        %inertialVelocity
    assignin(mdlWks,'v_icrf_input',inertial_v);
        %inertialPosition
    assignin(mdlWks,'x_icrf_input',inertial_x);
        %estimation state
    assignin(mdlWks,'x_input',[q;w]);
        %the torque from last iteration
    assignin(mdlWks,'prev_torque_input', torq);
        %julian date
    assignin(mdlWks,'jd_input',jd)
        %attitude with respect to fix frame
    assignin(mdlWks,'q_ff_in',q_ff)


    %reinitialize parameters for the theoratical
        %julianDate
    set_param(pathSpace, "startDate",mat2str(jd));
        %inertialPosition
    set_param(pathSpace, "inertialPosition",mat2str(inertial_x));
    %get_param(mdl+'/Spacecraft Dynamics', "inertialPosition");
        %inertialVelocity
    set_param(pathSpace, "inertialVelocity",mat2str(inertial_v));
    %get_param(mdl+'/Spacecraft Dynamics', "inertialVelocity");
        %attitude
    attitude = transpose(q);
    set_param(pathSpace, "attitude", mat2str(attitude));
        %attitudeRate
    rate = transpose(w);
    set_param(pathSpace, "attitudeRate", mat2str(rate));
    
    %reinitialize parameters for the measurement
        %julianDate
    set_param(pathSpace_mea, "startDate",mat2str(jd_mea));
        %inertialPosition
    set_param(pathSpace_mea, "inertialPosition",mat2str(inertial_x_mea));
        %inertialVelocity
    set_param(pathSpace_mea, "inertialVelocity",mat2str(inertial_v_mea));
        %attitude
    attitude_mea = transpose(q_mea);
    set_param(pathSpace_mea, "attitude", mat2str(attitude_mea));
        %attitudeRate
    rate_mea = transpose(w_mea);
    set_param(pathSpace_mea, "attitudeRate", mat2str(rate_mea));

    %simulation
    output = sim(simIN);

    %get output results
    % https://www.mathworks.com/help/simulink/slref/simulink.sdi.datasetref.getelement.html
    GDERun = Simulink.sdi.Run.getLatest;
    DSRef= getDatasetRef(GDERun);

    
    %port "x_icrf_out" 
    inertial_x = getOutport(DSRef,"x_icrf_out");

    %port "v_ircf_out"
    inertial_v= getOutport(DSRef,"v_icrf_out");

    %port "x_out"
    x = getOutport(DSRef,"x_out");
    q = x(1:4);
    w = x(5:7);

    %port "q_ff_out"
    q_ff = getOutport(DSRef,"q_ff_out");

    %port "torque_out"
    torq = getOutport(DSRef,"torque_out");

    %port "q_rotate_out"
    q_rotate = getOutport(DSRef,"q_monkey_out").';

    %port "theta_err_out"
    theta_err = getOutport(DSRef,"theta_err_out");
    
    %port "w_err_out"
    omega_err = getOutport(DSRef,"w_err_out");
    
    %port "w_right_frame" (angular rate of body frame with respect to inertial frame
    w_b2in = getOutport(DSRef,"w_right_frame");

    % log outputs
    x_list(:,counter)=x;
    q_rotate_list(:,counter)=q_rotate;
    torq_list(:,counter)=torq;
    w_body2inertial_list(:,counter) = w_b2in;
    theta_err_list(:,counter)=theta_err;
    w_err_list(:,counter) = norm(omega_err);
    counter = counter+1;

    %port "jy_out" julian date
    jd = getOutport(DSRef,"jy_out");


    %measurement system
    %port "x_icrf_mea_out"
    inertial_x_mea = getOutport(DSRef,"x_icrf_mea_out");

    %port "v_ircf_mea_out"
    inertial_v_mea = getOutport(DSRef,"v_icrf_mea_out");

    %port "q_mea_out" and "q_mea_out"
    q_mea = getOutport(DSRef,"q_mea_out");
    w_mea = getOutport(DSRef,"w_mea_out");


    %port "jy_mea_out"
    jd_mea = getOutport(DSRef,"jy_mea_out");

end

%outputting graphs (note: not every logged values are graphed, but it easy to graph more
time_axis = linspace(stepSize,stepSize*num_iterations,num_iterations);
figure(5)
plot(time_axis,theta_err_list,'DisplayName','No Kalman');
title('theta error, rad');
hold on;
figure(6)
plot(time_axis,w_err_list,'DisplayName','No Kalman');
title('omega error');
xlabel('time');
ylabel('norm of omega err vector, rad');
hold on;

% the function to get the output values from the model signals in each iteration
function v = getOutport(DSRef,portname)
    [el,difName,difIndex] = getElement(DSRef,portname);
    dimension = ndims(el.Values.Data);
    if(dimension==1)
        v=el.Values.Data(end);
    elseif (dimension == 2)
        v = el.Values.Data(:, end);
    elseif (dimension==3)
        v = el.Values.Data(:,:,end);
    end
    %[row,col] = size(el.Values.Data)
end