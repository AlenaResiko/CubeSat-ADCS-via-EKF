# CubeSat-ADCS-via-EKF

This is a MATLAB Simulink based model of CubeSat Attitude Determination and Control System.
Development of an Attitude Determination and Control System for CubeSats via Extended Kalman Filter

Paper link: (to be added).

This code was developed by Alena Chan and Ruimian Zheng.
Authors can be reached at alena.chan@columbia.edu and ruimian.zheng@columbia.edu.

This simulation should be run from 'Script_Multiplicative_FullSystem.m' file which calls 'Multiplicative_Kalman_FullSystem.slxc'.

Attached are figures showing that our EKF improve accuracy of attitude determination.

This software was developed in MATLAB, The MathWorks Inc. (2024). Optimization Toolbox version: 9.4 (R2024b), Natick, Massachusetts: The MathWorks Inc. https://www.mathworks.com
Note that this requires 2024b license.

[to add - video demo]

Note:
- The noise of the measurement should be set to higher value, since otherwise Kalman filter will rely on measurement. Hence for low values of R Kalman Filter doesn't change much the predicted value as has almost no impact on accuracy.
