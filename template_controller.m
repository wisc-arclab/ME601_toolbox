function [F, M, att] = template_controller(x,Y,params)
%TEMPLATE_CONTROLLER This is a template for the controller
%students of ME 601 will be developing.
% [F, M, att] = TEMPLATE_CONTROLLER(x,Y) returns the control inputs.
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

F = params.mass*params.grav;
M = zeros(3,1);
att = zeros(3,1);