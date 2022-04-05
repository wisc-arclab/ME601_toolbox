%% INSTRUCTOR USE ONLY
% This file will be used by the TA to test student controllers
% and trajectories in the lab session and record experimental
% data.
%
% This file is to be run in arc-desktop-03.me.wisc.edu
%
% Note that this node depends on arclab_quadcopters 
% which is a private ROS Package by ARC Lab.


%% Student Code Setup
% Change the name of PID_controller and traj_fi8_20s.csv
% to match the student files
controlhandle = @PID_controller;
trajfile = 'traj_fig8_20s.csv';

%% Copy csv file to resources folder
[~,arc_path] = system('rospack find arclab_quadcopters'); % Assumes Ubuntu
arc_csv_filepath = [arc_path(1:end-1),'/resources/traj/traj_me601.csv'];
system(strcat('cp ',trajfile,' ',arc_csv_filepath)); % Copy csv file

%% Global parameters
params = crazyflie();
global x Y     % define global state variable
x = zeros(12,1);
Y.y = zeros(4,1);
Y.dy = zeros(4,1);
Y.ddy = zeros(4,1);
Y.dddy = zeros(4,1);
Y.ddddy = zeros(4,1);


%% ROS Init
% after roscore is running already
try
    rosinit;
catch exp   % Error from rosinit
    disp("ROS_MASTER already running, skipping rosinit...")
end
rate = rosrate(100); %
addpath('/filespace/w/walters/catkin_ws/src/matlab_msg_gen_ros1/glnxa64/install/m')

% Subscribe to /crazyflie/state
msg_u = rosmessage("arclab_quadcopters/CMD_Att","DataFormat",'struct');
pub_u = rospublisher('/me601/u','arclab_quadcopters/CMD_Att',"DataFormat","struct");
sub_traj = rossubscriber('/trajectory','arclab_quadcopters/Trajectory',@traj_callback,"DataFormat","struct");
sub_state = rossubscriber('/crazyflie/state','arclab_quadcopters/QuadState',{@state_callback, pub_u, msg_u, controlhandle, params},"DataFormat","struct");


%% Functions
function [] = state_callback(~,msg,pub_u,msg_u,controlhandle,params)
    global x Y
    x(1) = msg.X;
    x(2) = msg.Y;
    x(3) = msg.Z;
    x(4) = msg.Dx;
    x(5) = msg.Dy;
    x(6) = msg.Dz;
    x_temp = x;
    Y_temp = Y;
    [F, ~, att] = controlhandle(x_temp,Y_temp,params);
    u = [F/(params.mass + 0.002); att]; % Thrust hack
    send(pub_u,populate_u(msg_u, u));
end

function [] = traj_callback(~,msg)
    global Y
    Y.y     = [msg.Pos.X; msg.Pos.Y; msg.Pos.Z; msg.Psi];
    Y.dy    = [msg.Vel.X; msg.Vel.Y; msg.Vel.Z; msg.Dpsi];
    Y.ddy   = [msg.Acc.X; msg.Acc.Y; msg.Acc.Z; 0];
    Y.dddy  = [msg.Jerk.X; msg.Jerk.Y; msg.Jerk.Z; 0];
    Y.ddddy = [msg.Snap.X; msg.Snap.Y; msg.Snap.Z; 0];
end

function msg = populate_u(msg,u)
    msg.Thrust = u(1);
    msg.Attitude.X = u(2);
    msg.Attitude.Y = u(3);
    msg.Attitude.Z = u(4);
end

function [] = make_traj_csv(traj_handle,filepath)
    [~,t_f] = traj_handle(0);
    freq = 100;
    rows = 1+t_f*freq;
    t = linspace(0,t_f,rows);
    MAT = zeros(rows, 18);
    for i = 1:rows
        [Y,~] = traj_handle(t(i));
        MAT(i,:) = [t(i);Y.y;Y.dy;Y.ddy(1:3);Y.dddy(1:3);Y.ddddy(1:3)]';
    end
    writematrix(MAT,filepath);
end





