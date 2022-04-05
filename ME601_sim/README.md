# ME 601 Quadcopter Simulator

This code simulates the dynamics of a quadcopter and is used to test student trajectory generation and controller code before implementing them in hardware. Execute the `runsim.m` file to run the simulator. This file requires the names of the student's trajectory generation and controller MATLAB files, as explained below.

## Provided by the Student
The student needs to create a trajectory generation function (for example called `example_trajectory.m`) and a controller function (for example called `example_controller.m`). These file names then should be added to the beginning of the `runsim.m` file in the following ways:
```
% trajectory generator function handle
trajhandle = @example_trajectory;

% controller function handle
controlhandle = @example_controller;
```
For more information regarding MATLAB function handles as used above, refer to [Create Function Handle](https://www.mathworks.com/help/matlab/matlab_prog/creating-a-function-handle.html).

### Trajectory and Controller files
To write the trajectory and controller function files, refer to the `template_trajectory.m` and `template_controller.m` files to understand the expected arguments and outputs.

## How the Simulator Works
The file `runsim.m` uses the controller and trajectory function handles provided by the student (see previous section) in order to simulate the dynamics of a quadrotor using the ode45 differential equation solver. The program will provide a real-time visualization of the quadrotor simulation, as well as plots of the position and velocities once the simulation is complete.
