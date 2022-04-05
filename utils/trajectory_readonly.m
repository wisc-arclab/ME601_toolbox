function [Y,t_f] = trajectory_readonly(t)
%TRAJECTORY_READONLY This is a function to read the given CSV file and 
% return the appropriate structure.
% [Y, t_f] = TRAJECTORY_READONLY(x,Y) returns flat output information Y and
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
%    
% NOTE: the simulator will spawn the robot to be at the
%       position returned for t == 0 (first row of M)

global M; % This variable should contain all the information in the csv file


% t_f is the final time of the trajectory. The domain of your
% trajectory function needs to be [0, t_f].
t_f = M(end,1);

% Get row corresponding to given time t
tr = round(t,2); % round given time t to closest multiple of 0.01
row = int64(tr/0.01)+1; % get row that is closest to given time t

% Extract desired row
pos = M(row,2:4)';
yaw = M(row,5);
vel = M(row,6:8)';
yawdot = M(row,9);
acc = M(row,10:12)';
jerk = M(row,13:15)';
snap = M(row,16:18)';

% Structure
Y.y     = [pos(:); yaw];
Y.dy    = [vel(:); yawdot];
Y.ddy   = [acc(:); 0];
Y.dddy  = [jerk(:); 0];
Y.ddddy = [snap(:); 0];

end
