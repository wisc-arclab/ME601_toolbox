function [Y,t_f] = template_trajectory(t)
%TEMPLATE_TRAJECTORY This is a template for the trajectory
%students of ME 601 will be developing.
% [Y, t_f] = TEMPLATE_TRAJECTORY(x,Y) returns flat output information Y and
% final time t_f.
%
% FUNTION RETURNS
% The variable t_f is the final time of the trajectory.
% Y is a struct containing the flat output and its derivatives, and has the
% following fields:
% Y.y     = [x; y; z; psi]
% Y.dy    = [dx; dy; dz; dpsi]
% Y.ddy   = [ddx; dddy; dddz; 0]
% Y.dddy  = [dddx; dddy; dddz; 0]
% Y.ddddy = [ddddx; ddddy; ddddz; 0]
% 
% FUNCTION ARGUMENTS
% This function takes any time from 0 to t_f as an argument.
%
% NOTE: This function needs to return in less than 0.01 seconds


% =================== Your code goes here ===================
% You have to set the pos, vel, acc, jerk, snap yaw and yawdot variables
%    
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0

pos = [0; 0; 0];
vel = [0; 0; 0];
acc = [0; 0; 0];
jerk = [0; 0; 0];
snap = [0; 0; 0];
yaw = 0;
yawdot = 0;

% t_f is the final time of the trajectory. The domain of your
% trajectory function needs to be [0, t_f].
t_f = 10;

% =================== Your code ends here ===================

Y.y     = [pos(:); yaw];
Y.dy    = [vel(:); yawdot];
Y.ddy   = [acc(:); 0];
Y.dddy  = [jerk(:); 0];
Y.ddddy = [snap(:); 0];

end
