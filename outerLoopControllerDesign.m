%% Outer Loop Controller Design
% Use a Root Locus plot to tune the gain of the outer loop proportional
% controller.

%% Initialize Robot Dimensions
r = 2.9527;      %[in]
d = 9.6456;      %[in]

%% Initialize Model Variables
% These model variables are taken from the tuned stepExperimentInitialize.m
% model variables.

K_rho = 16.85/4;
sigma_rho = 6.667;

K_phi = 2.92/4;
sigma_phi = 6.667;


%% Root Locus Values
% These values are from the rotational PI controller in
% robotPIDController.slx
K_p_phi = 1.46314969129408;
K_i_phi = 1.3275028377788;

% These values are from the forward PI controller in robotPIDController.slx
K_p_rho = 0.25355472395126;
K_i_rho = 0.230047969514189;

%% Use Root Locus plot to Design Outer Loop Controller
% sys1 is the Plant G(s) from the outer loop rotation controller.
% sys2 is the Plant G(s) from the outer loop forward controller.
% 
% Gain determined by root locus is outer loop proportional control gain of
% the rotational velocity control.
% 
% Each outer loop controller was given a gain of 4.
sys1 = tf([K_p_phi*K_phi*sigma_phi, K_i_phi*K_phi*sigma_phi],[1, sigma_phi*(1 + K_p_phi*K_phi), K_i_phi*K_phi*sigma_phi, 0]);
figure();
rlocus(sys1);

sys2 = tf([K_p_rho*K_rho*sigma_rho, K_i_rho*K_rho*sigma_rho],[1, sigma_rho*(1 + K_p_rho*K_rho), K_i_rho*K_rho*sigma_rho, 0]);
figure();
rlocus(sys2);