clear
clc

addpath('../Core') % references ROS interface and arm controller files you'll need for every lab

% start ROS 
con = rosStart(true); % start ROS with gripper enabled

% add command

% Target 1
T0e = [[   0.019,    0.969,    0.245,   47.046];[   0.917,   -0.115,    0.382,   73.269];[   0.398 ,   0.217,   -0.891,  100.547];[   0.,       0. ,      0.,       1.]];

% Target 2
%T0e = [[  -0.993,   -0.,       0.119,  -96.936];[   0.,      -1.,      -0.,       0.   ];[   0.119,    0.,       0.993,  401.229];[   0. ,      0.  ,     0.  ,     1.   ]];

% Target 3
%T0e =[ [-0.3409003, -0.1074855,  0.9339346, 282.96];[0.7842780, -0.5802868,  0.2194888, -48.302];[0.5183581,  0.8072881,  0.2821184, 235.071 ]; [0,0,0,1]];

% Target 4
%T0e =[[  0.5054096, -0.8370580, -0.2095115, -45];[-0.0305796,  0.2252773, -0.9738147,-300];[0.8623375,  0.4985821,  0.0882604, 63 ];[0,0,0,1]];

[q, isPos] = calculateIK(T0e);
if ~isempty(q)
    con = add_command(con,[q(1,:), 10]);
    disp("Now the current state value is : ...");
    disp(con.cur_state);
    disp("Finish the command!");


    [~,T0e_sim]=checkFK(con);

    % Report simulation positions and predicted positions
    disp("Simulation T0e = ")
    disp(T0e_sim)
end
disp("Target T0e = ")
disp(T0e)
disp("isPos =")
disp(isPos)
disp("q =")
disp(q)

%%
% shut down ROS
rosEnd; 