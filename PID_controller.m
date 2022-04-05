function [F, M, att] = PID_controller(x,Y,params)
%PID_CONTROLLER This is a PID controller example.
% [F, M, att] = PID_CONTROLLER(x,Y) returns the control inputs.
%
% FUNCTION RETURNS
% F is the thrust as a scalar (N). M is torque (Nm) as a
% 3x1 vector. The variable att is attitude (rad) as a 3x1 vector.
%
% FUNTION ARGUMENTS
% The control inputs are calculated as a function of x and Y. x is the
% state, and Y is a struct containing the flat output and its derivatives.
% The argument params is a struct containing information about the
% quadcopter system. See crazyflie.m for these parameter details.
% 
% x is [12x1] and contains variables 
% [x, y, z, xd, yd, zd, phi, theta, psi, p, q, r]
% Y is a struct containing the following fields:
% Y.y     = [x; y; z; psi]
% Y.dy    = [dx; dy; dz; dpsi]
% Y.ddy   = [ddx; dddy; dddz; 0]
% Y.dddy  = [dddx; dddy; dddz; 0]
% Y.ddddy = [ddddx; ddddy; ddddz; 0]
%
% NOTE: This function needs to return in less than 0.01 seconds

%% Gains
Kpos_p = diag([.5 .5 1.5]); % proportional position gain
Kpos_d = diag([1 1 2]); % derivative position gain
Katt_p = 5*diag([20 20 .2]); % proportional attitude gain
Katt_d = diag([20 20 1.2]); % derivative attitude gain

%% Postition Controller
e_pos = x(1:3) - Y.y(1:3);
e_vel = x(4:6) - Y.dy(1:3);
r_ddot_des = Y.ddy(1:3) - Kpos_p*e_pos - Kpos_d*e_vel;
F = params.mass*(r_ddot_des(3) + params.grav);

%% Attitude Controller
att = zeros(3,1);
att(1) = ((r_ddot_des(1)*sin(Y.y(4))) - r_ddot_des(2)*cos(Y.y(4)))/params.grav;
att(2) = ((r_ddot_des(1)*cos(Y.y(4))) + r_ddot_des(2)*sin(Y.y(4)))/params.grav;
att(3) = Y.y(4);

%% Torque Controller
e_att = x(7:9) - att;
omega_des = [0;0;Y.dy(4)]; % You can use differential flatness to improve this
e_omega = x(10:12) - omega_des;
att_ddot_des = - Katt_p*e_att - Katt_d*e_omega;
M = params.I*att_ddot_des;