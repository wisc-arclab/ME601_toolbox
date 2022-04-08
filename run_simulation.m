% This simulator is adapted from the U Penn MEAM 620 Robotics course:
% https://alliance.seas.upenn.edu/~meam620/wiki/index.php
%
% ***************** ME 601 QUADROTOR SIMULATION *****************
% clear workspace and add required paths
clear; close all; clc
addpath('utils')

%% ============ Your code goes here ============================

% Select the csv file with your trajectory (see trajectory_generator.m)
trajfile = 'traj_spiral_10s.csv';

% controller function handle
controlhandle = @PID_controller;

%% ============ Your code ends here ============================


% real-time: if set to true, the program will slow execution to 
% match real time.
real_time = true;

% Switch between planar (2D) and 3D quadcopter view
view_type = 3; % 2 for 2D, 3 for 3D

% number of quadrotors
nquad = 1;

% Load trajectory csv
global M
M = readmatrix(trajfile); % load in trajectory matrix
trajhandle = @trajectory_readonly;
% final time
[~,t_f] = trajhandle(0); % Obtain the domain [0,t_f] of the trajectory

% parameters for simulation
params = crazyflie();


%% **************************** FIGURES *****************************
fprintf('Initializing figures...\n')
h_fig = figure;
h_3d = gca;
axis equal
grid on
if view_type == 2
    view(0,0); % 2D x-z view
elseif view_type == 3
    view(3); % 3D view
end
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(nquad);

set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
max_iter  = 5000;      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
err = []; % runtime errors
for qn = 1:nquad
    % Get start and stop position
    des_start = trajhandle(0);
    des_stop  = trajhandle(t_f);
    stop{qn}  = des_stop.y(1:3);  % this is the goal state
    x0{qn}    = init_state( des_start.y(1:3), 0 );
    xtraj{qn} = zeros(max_iter*nstep, length(x0{qn}));
    ttraj{qn} = zeros(max_iter*nstep, 1);
end

x         = x0;        % state

pos_tol   = 0.01;
vel_tol   = 0.01;

%% ************************* RUN SIMULATION *************************
fprintf('Simulation Running....')
% Main loop
for iter = 1:max_iter

    timeint = time:tstep:time+cstep; % The time interval for ode45 to simulate

    tic;
    % Iterate over each quad
    for qn = 1:nquad
        % Initialize quad plot
        if iter == 1
            QP{qn} = QuadPlot(qn, x0{qn}, 0.1, 0.04, quadcolors(qn,:), max_iter, h_3d);
            desired_state = trajhandle(time);
            QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.y(1:3); desired_state.dy(1:3)], time);
            h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
        end

        % Run simulation
        UPenn_controller = @controller_readonly;
        [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, qn, UPenn_controller, trajhandle, params, controlhandle), timeint, x{qn});
        x{qn}    = xsave(end, :)';
        
        % Save to traj
        xtraj{qn}((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
        ttraj{qn}((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

        % Update quad plot
        desired_state = trajhandle(time + cstep);
        QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.y(1:3); desired_state.y(1:3)], time + cstep);
        set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
    end
    time = time + cstep; % Update simulation time
    t = toc;
    % Check to make sure ode45 is not timing out
    if(t> cstep*50)
        err = 'Ode45 Unstable';
        break;
    end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    % Check termination criteria
    if terminate_check(x, time, stop, pos_tol, vel_tol, t_f)
        break
    end
end

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
for qn = 1:nquad
    xtraj{qn} = xtraj{qn}(1:iter*nstep,:);
    ttraj{qn} = ttraj{qn}(1:iter*nstep);
end

% Plot the saved position and velocity of each robot
for qn = 1:nquad
    % Truncate saved variables
    QP{qn}.TruncateHist();
    % Plot position for each quad
    h_pos{qn} = figure('Name', ['Quad ' num2str(qn) ' : position']);
    plot_state(h_pos{qn}, QP{qn}.state_hist(1:3,:), QP{qn}.time_hist, 'pos', 'vic');
    plot_state(h_pos{qn}, QP{qn}.state_des_hist(1:3,:), QP{qn}.time_hist, 'pos', 'des');
    % Plot velocity for each quad
    h_vel{qn} = figure('Name', ['Quad ' num2str(qn) ' : velocity']);
    plot_state(h_vel{qn}, QP{qn}.state_hist(4:6,:), QP{qn}.time_hist, 'vel', 'vic');
    plot_state(h_vel{qn}, QP{qn}.state_des_hist(4:6,:), QP{qn}.time_hist, 'vel', 'des');
end
if(~isempty(err))
    error(err);
end

fprintf('finished.\n')
