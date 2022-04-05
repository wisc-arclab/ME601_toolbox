function [F, M, att] = LQR_controller(x,Y,params)
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

% Original:
% F = params.mass*params.grav;
% M = zeros(3,1);
% att = zeros(3,1);

%% LQR:
% Gains
Katt_p = 0.1*5*diag([1 1 .2]); % proportional attitude gain
Katt_d = 10*diag([2 2 1.2]); % derivative attitude gain
K = [0.0000    0.0000   31.6228   -0.0000    0.0000    7.2128
    0.0000   -1.0000    0.0000    0.0000   -0.2377    0.0000
    0.7071    0.0000    0.0000    0.2337    0.0000    0.0000
   -0.0000    0.0000    0.0000   -0.0000    0.0000    0.0000];
% linearization of 6-dim quadcopter dynamics
% A = [zeros(3), eye(3);
%      zeros(3), zeros(3)];
% B = (1/params.mass)*[zeros(3,4);
%      0,  0, params.grav, 0;
%      0, -params.grav, 0, 0;
%      1,  0, 0, 0];
%  
% % Implement LQR
% Q = diag([10,20,100,1,1,5]);
% R = diag([0.1,20,20,40]);
% [K,~,~] = lqr(A,B,Q,R);
ue = [params.grav*params.mass;0;0;0]; % control at equilib point
u = ue - K*(x(1:6) - [Y.y(1:3);Y.dy(1:3)]);
F = u(1);
att = u(2:4);

% Torque Controller
e_att = x(7:9) - att;
omega_des = [0;0;Y.dy(4)]; % differential flatness for students
e_omega = x(10:12) - omega_des;
att_ddot_des = - Katt_p*e_att - Katt_d*e_omega;
M = params.I*att_ddot_des;

% e_att = x(7:9) - att;
% omega_bar = Katt_p*e_att;
% e_omega = x(10:12) - omega_bar;
% alpha = Katt_d*e_omega;
% M = params.I*alpha;

