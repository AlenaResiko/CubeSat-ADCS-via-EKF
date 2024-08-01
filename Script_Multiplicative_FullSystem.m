clear all; %clear all variables from previous runs

%save simulation outputs to produce results
num_iterations = 100; % number of iteration of the simulation to run
x_list = zeros(7,num_iterations); % time series of the estimated state vector 
s_list = zeros(7,7,num_iterations); % time series of the estimated covariance
q_rotate_list = zeros(4,num_iterations); % time series of the quaternion offset from target
torq_list = zeros(3,num_iterations); % time series of torque implemented
theta_err_list = zeros(1,num_iterations); %time series of the angle estimation error (estimation theta - true theta)
w_err_list = zeros(1,num_iterations); % time series of the norm of the angular rate estimation error (norm of (estimation w - true w))
w_body2inertial_list = zeros(3,num_iterations); % time series of the angular rate estimation of body frame with respect to inertial frame
counter = 1; %index for logging the timeseries

%getting the model workspace for modifying variables and simulation
mdl = "Multiplicative_Kalman_FullSystem"; 
simIN = Simulink.SimulationInput(mdl); 
pathSpace = mdl+'/Orbit sim'+'/Spacecraft Dynamics';
pathSpace_mea = mdl+ '/generateMeasurement' + '/Orbit sim measurement' + '/Spacecraft Dynamics';
mdlWks = get_param('Multiplicative_Kalman_FullSystem','ModelWorkspace');


%set the time to run one iteration
stepSize = 1; 
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


%initialize the constant inputs
s =[[1.94870414, 1.25588909 ,1.4416761, 1.0780205, 1.49168854, 1.53894753, 1.37981514];
    [1.25588909 ,2.30599344, 1.57766712, 1.62980068, 1.46050557, 1.31460547,1.13813509];
    [1.4416761, 1.57766712, 1.70958392, 1.12140846, 1.38322267, 1.58836736, 1.34392315];
    [1.0780205, 1.62980068, 1.12140846, 1.77837393 ,1.21690337, 0.94427956 ,1.46043147];
    [1.49168854,1.46050557, 1.38322267, 1.21690337, 2.06327445 ,1.39185931, 1.33424815];
    [1.53894753, 1.31460547, 1.58836736 ,0.94427956, 1.39185931, 2.02772691, 1.7277207];
    [1.37981514, 1.13813509 ,1.34392315, 1.46043147, 1.33424815, 1.7277207, 2.16380015]]; %initial covariance

q_ff = [0.76813986749607;-0.00060774689776115;0.00073833382166474;-0.64028136742373]; %initial attitude with respect to fix frame

%noise covariances
R = 0.5*eye(6); 
Q = 0.05*eye(7);
assignin(mdlWks,'NoiseCov',R)
assignin(mdlWks,'Q_identity',Q)

m=100; % mass of CubeSat
assignin(mdlWks,'m',m);

%call the simulation for the assigned number of iterations
for i=(1:num_iterations) 
    %reassign the variables according to the end result of last iteration

        %inertialVelocity
    assignin(mdlWks,'v_icrf_input',inertial_v);
        %inertialPosition
    assignin(mdlWks,'x_icrf_input',inertial_x);
        %estimation covariance
    assignin(mdlWks,'s_input', s);
        %estimation state
    assignin(mdlWks,'x_input',[q;w]);
        %the torque from last iteration
    assignin(mdlWks,'prev_torque_input', torq);
        %julian date
    assignin(mdlWks,'jd_input',jd)
    assignin(mdlWks,'q_ff_in',q_ff)

    %reinitialize parameters for the theoratical model
        %julianDate
    set_param(pathSpace, "startDate",mat2str(jd));
        %inertialPosition
    set_param(pathSpace, "inertialPosition",mat2str(inertial_x));
        %inertialVelocity
    set_param(pathSpace, "inertialVelocity",mat2str(inertial_v));
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

    %port "s_out"
    s = getOutport(DSRef,"s_out");

    %port "q_ff_out"
    q_ff = getOutport(DSRef,"q_ff_out");

    %port "torque_out"
    torq = getOutport(DSRef,"torque_out");

    %port "q_rotate_out"
    q_rotate = getOutport(DSRef,"q_rotate_out").';
    
    %port "theta_err_out"
    theta_err = getOutport(DSRef,"theta_err_out");
    
    %port "w_err_out"
    omega_err = getOutport(DSRef,"w_err_out").';

    %port "w_right_frame" (angular rate of body frame with respect to inertial frame
    w_b2in = getOutport(DSRef,"w_right_frame");

    % log outputs
    x_list(:,counter)=x; 
    q_rotate_list(:,counter)=q_rotate;
    torq_list(:,counter)=torq;
    s_list(:,:, counter) = s;
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
plot(time_axis,theta_err_list,'DisplayName','Kalman');
figure(6)
plot(time_axis,w_err_list,'DisplayName','Kalman');
legend

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
end
