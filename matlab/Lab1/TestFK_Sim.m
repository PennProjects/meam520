clear
clc

addpath('../Core') % references ROS interface and arm controller files you'll need for every lab

% start ROS 
con = rosStart(true); % start ROS with gripper enabled

% add command
q = [0 0 0 0 0 0];

con = add_command(con,q);
disp("Now the current state value is : ...");
disp(con.cur_state);
disp("Finish the command!");

[jointPositions_sim,T0e_sim]=checkFK(con);
[jointPositions, T0e] = calculateFK(q);

% Report simulation positions and predicted positions
disp("Simulation Joint Positions = ")
disp(jointPositions_sim)
disp("Predicted Joint Positions = ")
disp(jointPositions)
disp("Simulation T0e = ")
disp(T0e_sim)
disp("Predicted T0e = ")
disp(T0e)


%%
% shut down ROS
rosEnd; 