    %NEXT STEP: remember to input the inertial position everytime
clear all;

% save simulation outputs
num_iterations = 100;
x_list = zeros(7,num_iterations);
s_list = zeros(7,7,num_iterations);
q_rotate_list = zeros(4,num_iterations);
q_rotate_list1 = zeros(4,num_iterations);
torq_list = zeros(3,num_iterations);
theta_err_list = zeros(1,num_iterations);
w_err_list = zeros(1,num_iterations);
w_dir_err_list = zeros(3,num_iterations);
w_body2inertial_list = zeros(3,num_iterations);
q_mea_list = zeros(4,num_iterations);
counter = 1;
x_input_list = zeros(7,num_iterations);

mdl = "Kalman_FullSystem";
simIN = Simulink.SimulationInput(mdl);
pathSpace = mdl+'/Orbit sim'+'/Spacecraft Dynamics';
pathSpace_mea = mdl+ '/generateMeasurement' + '/Orbit sim measurement' + '/Spacecraft Dynamics';
mdlWks = get_param('Kalman_FullSystem','ModelWorkspace');


%set the start/stop time
stepSize = 0.05;
set_param(mdl,'StopTime',mat2str(stepSize))
set_param(mdl,'SolverType','Fixed-step');
set_param(mdl,'FixedStep', mat2str(stepSize))
assignin(mdlWks,'t_step',(stepSize));

%initialize the very initial parameters for theoratical
jd =2458850;
inertial_x=[3649700.0, 3308200.0, -4676600.0];
inertial_v= [-2750.8, 6666.4, 2573.4];
w = [0.02;0.02;-0.01];
q = [0.2; -0.6 ;-0.5; -0.59160797831];
torq = [0;0;0];

%initialize the very initial parameters for measurement
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
    [1.37981514, 1.13813509 ,1.34392315, 1.46043147, 1.33424815, 1.7277207, 2.16380015]];

q_ff = [0.76813986749607;-0.00060774689776115;0.00073833382166474;-0.64028136742373];

     % %know the parameters
     %    configSet = getActiveConfigSet(mdl);
     %    configSetNames = get_param(configSet,"ObjectParameters"); 
theta_err_accum = 0;
omega_norm_err_accum = 0;
for i=(1:num_iterations) 
    assignin(mdlWks,'v_icrf_input',inertial_v);
    assignin(mdlWks,'x_icrf_input',inertial_x);
    assignin(mdlWks,'s_input', s);
    assignin(mdlWks,'x_input',[q;w]);
    x_input_list = [q;w];
    assignin(mdlWks,'prev_torque_input', torq);
    assignin(mdlWks,'controlRun', false);
    assignin(mdlWks,'savedTorque', torq);
    assignin(mdlWks,'jd_input',jd)
    assignin(mdlWks,'q_ff_in',q_ff)
    assignin(mdlWks,'q_mea',q_mea);
    assignin(mdlWks,'w_mea',w_mea);

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
    %get_param(mdl+'/Spacecraft Dynamics', "inertialPosition");
        %inertialVelocity
    set_param(pathSpace_mea, "inertialVelocity",mat2str(inertial_v_mea));
    %get_param(mdl+'/Spacecraft Dynamics', "inertialVelocity");
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

    q_rotate = getOutport(DSRef,"q_monkey_out").';
    q_rotate1 = getOutport(DSRef,"q_true_err").';
    theta_err = getOutport(DSRef,"theta_err_out");
    omega_err = getOutport(DSRef,"w_err_out");
    w_b2in = getOutport(DSRef,"w_right_frame");

    % log outputs
    x_list(:,counter)=x;
    q_rotate_list(:,counter)=q_rotate;
    q_rotate_list1(:,counter)=q_rotate1;
    torq_list(:,counter)=torq;
    s_list(:,:, counter) = s;
    w_body2inertial_list(:,counter) = w_b2in;

    theta_err_accum = theta_err_accum+theta_err;
    theta_err_list(:,counter)=theta_err;
    omega_norm_err_accum=omega_norm_err_accum+norm(omega_err);
    w_dir_err_list(:,counter) = omega_err;
    w_err_list(:,counter) = norm(omega_err);
    counter = counter+1;

    %port "jy_out" julian date
    jd_temp=jd;
    jd = getOutport(DSRef,"jy_out");
    % jd-jd_temp;


    %measurement system
    %port "x_icrf_mea_out"
    inertial_x_mea = getOutport(DSRef,"x_icrf_mea_out");

    %port "v_ircf_mea_out"
    inertial_v_mea = getOutport(DSRef,"v_icrf_mea_out");

    %port "q_mea_out" and "q_mea_out"
    q_mea = getOutport(DSRef,"q_mea_out");
    w_mea = getOutport(DSRef,"w_mea_out");
    q_mea_list(:,counter) = q_mea;

    %port "jy_mea_out"
    jd_mea_temp = jd_mea;
    jd_mea = getOutport(DSRef,"jy_mea_out");
    % jd_mea-jd_mea_temp;

end

%disp(x_list);
%disp(torq_list);
%disp(q_rotate_list);
%disp(s_list);
time_axis = linspace(stepSize,stepSize*num_iterations,num_iterations);

% visualization quaternions
theta_state_data = 2*acos(x_list(1,:));
theta_data = 2*acos(q_rotate_list(1,:));
trace_data = squeeze(s_list(1,1,:)+s_list(2,2,:)+s_list(3,3,:)+s_list(4,4,:)+s_list(5,5,:)+s_list(6,6,:)+s_list(7,7,:));
theta_data1 = 2*acos(q_rotate_list1(1,:));


torq1_data = torq_list(1,:).^2;
torq2_data = torq_list(2,:).^2;
torq3_data = torq_list(3,:).^2;
torq_norms_data = (torq1_data(:)+torq2_data(:)+torq3_data(:)).^(1/2);
% figure(1)
% plot(time_axis,torq_norms_data,"--b");
% title('torq norms scaled');
% figure(9)
% plot(time_axis,w_body2inertial_list);
% title('w_b2in');
% title('w body to inertial');
% figure(5)
% plot(time_axis,torq_list)
% title('torq')
%figure(1)
%plot(time_axis,q_mea_list(1:4,:))
% figure(2)
% plot(time_axis,theta_data,'DisplayName','est');
% hold on;
% plot(time_axis,theta_data1,'DisplayName','act');
% title('angle offset');
% hold off;
% figure(3)
% plot(time_axis,q_rotate_list(1,:),'DisplayName','est');
% hold on;
% plot(time_axis,q_rotate_list1(1,:),'DisplayName','act');
% title('angle offset 3d');
% hold off;
% % figure(3)
% % plot(time_axis,trace_data,"-ob");
% % title('trace');
% w1_data = x_list(5,:).^2;
% w2_data = x_list(6,:).^2;
% w3_data = x_list(7,:).^2;
% w_data = x_list(5:7,:);
% omega_norms_data = (w1_data(:)+w2_data(:)+w3_data(:)).^(1/2);
% figure(4)
% plot(time_axis,omega_norms_data,"-or");
% title('magnitude of angular velocity');
% 
% figure(6)
% plot(time_axis,w_data);
% title('w')

% figure(4)
% plot(time_axis,x_list(2,:),time_axis,x_list(3,:),time_axis,x_list(4,:));
figure(5)
plot(time_axis,theta_err_list);
title('theta error');
legend
hold off;
figure(6)
plot(time_axis,w_err_list,'DisplayName','Kalman');
legend
title('omega error');
hold off;
figure(8)
plot(time_axis, w_dir_err_list,'DisplayName','No Kalman');
legend;
hold off;
title('omega dir error');

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