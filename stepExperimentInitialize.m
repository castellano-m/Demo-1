%% Step Experiment Script
% The following MATLAB code will simulate the step response of both forward
% velocity and rotational velocity for the robot.

%% Initialize Data Array
leftVelocity = [1.00,0.00;1.05,2.90;1.10,3.49;1.15,4.31;1.20,4.70;1.25,4.70;
                1.30,5.91;1.35,5.52;1.40,6.21;1.50,5.67;1.55,6.33;1.60,5.71;
                1.65,5.99;1.70,5.71;1.75,5.95;1.80,6.21;1.85,6.21;1.90,7.85;
                1.95,6.10;2.00,5.99;2.05,5.88;2.10,5.88;2.15,8.61;2.20,6.33;
                2.25,5.88;2.30,6.33;2.35,6.68;2.40,6.33;2.45,6.25;2.50,5.74;
                2.55,6.29;2.60,6.17;2.65,6.17;2.70,6.42;2.75,6.10;2.80,6.14];
rightVelocity = [1.00,0.00;1.05,2.37;1.10,3.02;1.15,4.21;1.20,4.61;1.25,4.57;
                 1.30,5.95;1.35,5.67;1.40,5.39;1.50,5.39;1.55,5.61;1.60,6.02;
                 1.65,5.71;1.70,5.71;1.75,6.21;1.80,6.10;1.85,5.81;1.90,6.29;
                 1.95,6.25;2.00,5.91;2.05,6.91;2.10,6.17;2.15,6.63;2.20,6.06;
                 2.25,6.59;2.30,6.50;2.35,5.74;2.40,6.29;2.45,4.52;2.50,5.99;
                 2.55,6.17;2.60,4.93;2.65,5.48;2.70,5.81;2.75,6.02;2.80,5.95];
forwardVelocity = [1.00,0.00;1.02,5.88;1.05,6.24;1.08,7.15;1.10,7.84;
                   1.12,10.19;1.15,10.91;1.17,12.70;1.20,12.44;1.23,12.98;
                   1.25,14.31;1.27,15.14;1.30,14.68;1.33,14.95;1.35,16.33;
                   1.37,15.71;1.40,15.84;1.42,15.84;1.48,15.99;1.50,17.37;
                   1.52,17.20;1.55,16.15;1.58,16.80;1.60,16.80;1.63,16.80;
                   1.65,15.63;1.67,17.51;1.70,17.36;1.73,17.72;1.75,17.15;
                   1.77,17.06;1.80,17.05;1.83,15.88;1.85,17.36;1.88,16.44;
                   1.90,17.84;1.92,16.24;1.95,16.71;1.98,15.50;2.00,17.01;
                   2.03,16.52;2.05,16.45;2.08,17.55;2.10,16.77;2.13,16.43;
                   2.15,16.85;2.20,18.83;2.23,16.52;2.25,16.48;2.28,15.80;
                   2.30,16.85;2.33,16.61;2.35,16.51;2.38,16.56;2.40,16.42;
                   2.43,18.25;2.45,16.66;2.48,17.30;2.50,15.84;2.53,16.42;
                   2.55,16.80;2.58,16.52;2.60,16.51;2.63,16.95;2.65,17.00;
                   2.68,18.38;2.70,17.05;2.73,17.05;2.75,17.25;2.78,16.47];
rotationalVelocity = [1.00,0.00;1.02,-0.92;1.05,-1.04;1.08,-1.29;1.10,-1.50;
                      1.12,-1.70;1.15,-1.96;1.17,-2.30;1.20,-2.10;1.23,-2.36;
                      1.25,-2.48;1.27,-2.40;1.30,-2.71;1.33,-2.64;1.35,-2.52;
                      1.37,-2.60;1.40,-2.88;1.42,-2.41;1.48,-3.09;1.50,-2.59;
                      1.52,-2.95;1.55,-3.05;1.58,-2.72;1.60,-2.70;1.63,-2.70;
                      1.65,-2.54;1.67,-2.82;1.70,-3.08;1.73,-2.77;1.75,-2.89;
                      1.77,-2.82;1.80,-2.86;1.83,-2.92;1.85,-3.37;1.88,-3.14;
                      1.90,-2.95;1.92,-3.42;1.95,-2.95;1.98,-3.07;2.00,-3.07;
                      2.03,-3.13;2.05,-2.74;2.08,-2.91;2.10,-2.86;2.13,-3.32;
                      2.15,-2.91;2.20,-2.73;2.23,-2.90;2.25,-3.01;2.28,-2.79;
                      2.30,-3.09;2.33,-3.10;2.35,-2.99;2.38,-2.99;2.40,-2.80;
                      2.43,-3.04;2.45,-2.72;2.48,-2.96;2.50,-3.12;2.53,-2.90;
                      2.55,-3.26;2.58,-2.82;2.60,-2.86;2.63,-2.99;2.65,-2.70;
                      2.68,-3.18;2.70,-3.00;2.73,-2.58;2.75,-2.67;2.78,-2.89;
                      2.80,-3.07];

%% Initialize Robot Dimensions
r = 2.9527;      %[in]
d = 9.6456;      %[in]

%% Initialize Model Variables
% The model variables with _left and _right subscripts are used to model
% the behavior of the left and right motors separately when supplied with a
% 4 V step response.
% 
% The model variables with the _rho subscript models the robot's forward
% velocity response when each motor is supplied with a 4 V step input.
% 
% The model variables with the _phi subscript models the robot's rotational
% velocity response when one motor is supplied by a +4 V step and the other
% motor is supplied by a -4 V step input.

K_left = 5.80/4;
sigma_left = 8;

K_right = 5.83/4;
sigma_right = 8;

K_rho = 16.85/4;
sigma_rho = 6.667;

K_phi = 2.92/4;
sigma_phi = 6.667;

%% Run Simulation and Plot Results
% Simulate each motor's step response, the robot's forward velocity step
% response, and the robot's rotational velocity step response.
% 
% Plot experimental data against simulated data in order to tune model
% variables.

out1 = sim('leftMotorStepResponse');
out2 = sim('rightMotorStepResponse');
out3 = sim('fullRobotForward');
out4 = sim('fullRobotRot');

timeAng = leftVelocity(:,1);
leftAngVel = leftVelocity(:,2);
rightAngVel = rightVelocity(:,2);

timeLin = forwardVelocity(:,1);
forwardVel = forwardVelocity(:,2);

timeRot = rotationalVelocity(:,1);
rotationalVel = rotationalVelocity(:,2);

figure(1);
plot(out1.simout);  title("Left Wheel Step Response");
xlabel("time [s]");    ylabel("left wheel angular velocity [rads/s]");
hold on
plot(timeAng, leftAngVel);
hold off

figure(2);
plot(out2.simout1); title("Right Wheel Step Response");
xlabel("time [s]");    ylabel("right wheel angular velocity [rads/s]");
hold on
plot(timeAng, rightAngVel);
hold off

figure(3);
plot(out3.simout);  title("Forward Velocity Step Response");
xlabel("time [s]");     ylabel("Robot Velocity");
hold on
plot(timeLin, forwardVel);
hold off

figure(4);
plot(out4.simout); title("Rotational Velocity Step Response");
xlabel("time [s]");     ylabel("Rotational Velocity [rads/s]");
hold on
plot(timeRot, rotationalVel);
hold off