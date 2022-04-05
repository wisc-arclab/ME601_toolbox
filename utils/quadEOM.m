function sdot = quadEOM(t, s, qn, controlhandle, trajhandle, params, student_controller)
% QUADEOM Wrapper function for solving quadrotor equation of motion
% 	quadEOM takes in time, state vector, controller, trajectory generator
% 	and parameters and output the derivative of the state vector, the
% 	actual calcution is done in quadEOM_readonly.
%
% INPUTS:
% t             - 1 x 1, time
% s             - 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
% qn            - quad number (used for multi-robot simulations)
% controlhandle - function handle of your controller
% trajhandle    - function handle of your trajectory generator
% params        - struct, output from crazyflie() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot          - 13 x 1, derivative of state vector s
%
% NOTE: You should not modify this function
% See Also: quadEOM_readonly, crazyflie

% convert state to quad stuct for control
qd{qn} = stateToQd(s);

% Get desired_state
desired_state = trajhandle(t);

% The desired_state is set in the trajectory generator
qd{qn}.pos_des      = desired_state.y(1:3);
qd{qn}.vel_des      = desired_state.dy(1:3);
qd{qn}.acc_des      = desired_state.ddy(1:3);
qd{qn}.jerk_des     = desired_state.dddy(1:3);
qd{qn}.snap_des     = desired_state.ddddy(1:3);
qd{qn}.yaw_des      = desired_state.y(4);
qd{qn}.yawdot_des   = desired_state.dy(4);

% get control outputs
[F, M, trpy, drpy] = controlhandle(t, qd, qn, params, student_controller);

% compute derivative
sdot = quadEOM_readonly(t, s, F, M, params);

end
