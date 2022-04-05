%%%%
% Victor Freire <freiremelgiz@wisc.edu>
% ARC Lab Research Group <https://xu.me.wisc.edu/>
% University of Wisconsin-Madison
% Updated: April 2022
%
%
% ME 601 Homework 3: Trajectory Generation
% Generate minimum-snap quadcopter trajectories using
% differential flatness. This file also produces a
% .csv file with the correct format for the lab.
%
%
% References:
% D. Mellinger and V. Kumar, “Minimum snap trajectory generation and
%   control for quadrotors." 
% V. Freire and X. Xu, “Flatness-based quadcopter trajectory planning
%   and tracking with continuous-time safety guarantees."
%%%%
clear; clc; close all

%% User Parameters
% ====== Your code goes here ======

% If you choose aggressive (short) flight durations, your controller
% will probably fail. If you choose conservative (long) flight
% durations, your TA will get bored.
t_flight = 20 ; % [sec] Flight Duration

% Specify the "waypoints" for the trajectory in meters:
% waypoints = [x0, x1, ...;
%              y0, y1, ...;
%              z0, z1, ...]';

% Sample trajectory "Figure 8"
waypoints = [0, 1, 1, 0,-1,-1, 0;
             0, 1,-1, 0, 1,-1, 0;
             0, 1, 1, 1, 1, 1, 0];

% Objective order to minimize
k_r = 4; % Minimize snap (Try k_r = 1 for min-length. Which is better?)

% ====== Your code ends here ======
%% Other Parameters
n = 4; % Polynomial order (4 minimum)

% Polynomial parameters
m = size(waypoints,2); % Number of waypoints
t_bounds = linspace(0,t_flight,m); % uniform segments

% Solve for polynomial coefficients (QUADPROG)
% Notice that the three axes are de-coupled
polys_x = get_polys(waypoints(1,:), m, n, t_bounds, k_r); 
polys_y = get_polys(waypoints(2,:), m, n, t_bounds, k_r);
polys_z = get_polys(waypoints(3,:), m, n, t_bounds, k_r);


% Plot trajectory
[x_traj, y_traj, z_traj, t_traj] = draw(polys_x, polys_y, polys_z, waypoints, t_bounds);

% Save as .csv file
Y = to_csv(t_bounds, polys_x, polys_y, polys_z);

%% Helper functions
function polys = get_polys(waypoints, m, n, t_bounds, k_der)
    %% Find H and f
    mask_pos = ones(1,(n+1));
    % Find integrand
    integrand = mask_pos;
    for i = 1:k_der
        integrand = polyder(integrand);
    end
    H = [];
    for seg = 1:m-1
        H_part = zeros(n+1);
        % Construct H_part
        dt_exp = (n-k_der+1)*2-1;
        for k = 1:n-k_der+1
            H_part(end-k+1,end-k+1) = integrand(k)^2/dt_exp*(t_bounds(seg+1)^dt_exp - t_bounds(seg)^dt_exp); % Diag
            if k ~= 1
                iter = 1;
                for j = length(H_part)-k+2:length(H_part)
                    H_part(end-k+1,j) = 2*integrand(k)*integrand(length(H_part)-j+1)/(dt_exp+iter)*...
                        (t_bounds(seg+1)^(dt_exp+iter) - t_bounds(seg)^(dt_exp+iter)); % Off-diag
                    iter = iter + 1;
                end
            end
            dt_exp = dt_exp-2;
        end  
        H = blkdiag(H,H_part);
    end
    %disp(H);
    H = (1/2)*(H+H'); % Quadprog syntax
    %disp(H)
    f = zeros(length(H),1);

    %% QP Constraints
    Aeq = zeros((n+1)*(m-1));
    beq = zeros(length(Aeq),1);
    mask_vel = flip(polyder(ones(n+1,1)));
    mask_acc = flip(polyder(polyder(ones(n+1,1))));
    
    % Initial and final pos-vel-acc constraints (6 eq)
    Aeq(1,1:n+1) = get_tvec(t_bounds(1), n, 0); %Initial pos
    beq(1) = waypoints(1);
    Aeq(2,2:n+1) = mask_vel.*get_tvec(t_bounds(1), n, 1); %Initial vel
    Aeq(3,3:n+1) = mask_acc.*get_tvec(t_bounds(1), n, 2); %Initial acc
    Aeq(4,length(Aeq)-n:end) = get_tvec(t_bounds(end), n, 0); %Final pos
    beq(4) = waypoints(end);
    Aeq(5,length(Aeq)-n+1:end) = mask_vel.*get_tvec(t_bounds(end), n, 1); %Final vel
    Aeq(6,length(Aeq)-n+2:end) = mask_acc.*get_tvec(t_bounds(end), n, 2); %Final acc
    row = 6; % Number of equations used
    
    % Middle waypoint constraints (m-2 eq)
    for segment = 1:m-2
        row = row+1;
        Aeq(row,((n+1)*segment+1):(n+1)*(segment+1)) = get_tvec(t_bounds(segment+1), n, 0);
        beq(row) = waypoints(segment+1);
    end
    
    % Continuity constraints  ((m-2)*3 equations)
    for segment = 1:m-2
        row = row+1;
        Aeq(row,(n+1)*(segment-1)+1:(n+1)*(segment+1)) =...
            [get_tvec(t_bounds(segment+1), n, 0), -get_tvec(t_bounds(segment+1), n, 0)];
        row = row+1;
        Aeq(row,(n+1)*(segment-1)+1:(n+1)*(segment+1)) =...
            [0, mask_vel.*get_tvec(t_bounds(segment+1), n, 1), 0, mask_vel.*-get_tvec(t_bounds(segment+1), n, 1)];
        row = row+1;
        Aeq(row,(n+1)*(segment-1)+1:(n+1)*(segment+1)) =...
            [0, 0, mask_acc.*get_tvec(t_bounds(segment+1), n, 2), 0, 0, mask_acc.*-get_tvec(t_bounds(segment+1), n, 2)];
    end
 
    %% QuadProg
    options = optimoptions('quadprog','Display','off','MaxIter',4000);
    polys = quadprog(H,f,[],[],Aeq,beq,[],[],[],options);
    polys = flip(reshape(polys, [n+1 m-1]));
end
function prec = get_precision(Ts)
    Ts = abs(Ts); %in case of negative numbers
    prec = 0;
    while (floor(Ts*10^prec)~=Ts*10^prec)
        prec=prec+1;
    end
end
function [x_traj, y_traj, z_traj, t_traj] = draw(polys_x, polys_y, polys_z, waypoints, t_bounds)
    % Compute values for traj
    Ts = 0.01;
    precision = get_precision(Ts); % Find precision to round time intervals
    t_bounds = round(t_bounds,precision); % Round time intervals
    tsim = t_bounds(end);
    k = 1;
    for int = 1:length(t_bounds)-1
        % Iterate over time interval based on time step
        for t = t_bounds(int):Ts:t_bounds(int+1)-Ts
            x_traj(k) = polyval((polys_x(:,int)),t);
            y_traj(k) = polyval((polys_y(:,int)),t);
            z_traj(k) = polyval((polys_z(:,int)),t);
            k = k+1;
        end
    end
    x_traj(k) = polyval((polys_x(:,int)),t);
    y_traj(k) = polyval((polys_y(:,int)),t);
    z_traj(k) = polyval((polys_z(:,int)),t);
    t_traj = (t_bounds(1):Ts:t_bounds(end))';
    plot3(x_traj,y_traj,z_traj,'--k','LineWidth',1.5);
    hold on
    plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'or')
    legend('Trajectory','Waypoints','Location','best')
    grid on
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
end
function tvec = get_tvec(t, n, n_der)
    tvec = zeros(1,n+1-n_der);
    for i = 0:n-n_der
        tvec(i+1) = t^i;
    end
end
function r = to_csv(t_bounds, polys_x, polys_y, polys_z)
  % Save the trajectory in the right csv format:
  % time|x|y|z|psi|dx|dy|dz|dpsi|ddx|ddy|ddz|dddx|dddy|dddz|ddddx|ddddy|ddddz
  % Sample time
  Ts = 0.01; % 100 Hz
  % Get time vector
  t = (0:Ts:t_bounds(end))';
  % Compute flat outputs
  r = zeros(15,length(t));
  r_idx = 1;
  for seg = 1:length(t_bounds)-1 % Iterate over piece-wise segments
    t_seg = t(t >= t_bounds(seg) & t < t_bounds(seg+1));
    for r_der = 0:4 % Iterate over pos, vel, acc...
      x = polyderval(polys_x(:,seg),t_seg,r_der);
      y = polyderval(polys_y(:,seg),t_seg,r_der);
      z = polyderval(polys_z(:,seg),t_seg,r_der);
      r(3*r_der+1:3*(r_der+1),r_idx:r_idx + length(t_seg)-1) = [x';y';z'];
    end
    r_idx = r_idx + length(t_seg); % Update time index
  end
  % Final flat outputs
  for r_der = 0:4 % Iterate over pos, vel, acc...
      x = polyderval(polys_x(:,end),t_bounds(end),r_der);
      y = polyderval(polys_y(:,end),t_bounds(end),r_der);
      z = polyderval(polys_z(:,end),t_bounds(end),r_der);
      r(3*r_der+1:3*(r_der+1),end) = [x';y';z'];
  end
  
  % Write csv file
  writematrix([t, r(1:3,:)', zeros(length(t),1),...
            r(4:6,:)', zeros(length(t),1),...
            r(7:9,:)', r(10:12,:)', r(13:15,:)'],...
            fullfile('traj.csv'))
end
function val = polyderval(p,t,r)
    % p - coefs [p_1*t^n, p_2*t^{n-1}, ..., p_n*t, p_{n+1}]
    % t - evaluation points (time vector)
    % r - derivative order 0 <= r <= n
    for i = 1:r
        p = polyder(p);
    end
    val = polyval(p,t);
end

