%% Setup
clear all
close all
addpath('maps')
addpath('../Core')

%% Simulation Parameters

qStart = [0,0,0,0,0,0];
qGoal = [0,0,1.1,0,0,0];

map = loadmap('map1.txt');

% Get the path
[path] = potentialFieldPath(map, qStart, qGoal);

%start ROS
lynx = ArmController();
pause(1) % wait for setup
collision = false;

% Time in number of 0.1 sec intervals to wait before sending next command
wait_count = 50;

% iterate over target waypoints
for target_index = 1:length(path(:,1))
    q = path(target_index, :);
    disp("Goal:")
    disp(q)
    lynx.set_pos(q)
    reached_target = false;

    % Count is number of time steps waited 
    count = 0;
    while ~reached_target
        % Check if robot is collided then wait
        collision = collision | lynx.is_collided();
        pause(0.1)

        % iterate count and check if should send next command
        count = count + 1;
        if count > wait_count
            reached_target = true;
            count = 0;
        end

    end
    % End control loop

    disp("Current Configuration:");
    [pos, vel] = lynx.get_state();
    disp(pos);
end
if collision
    disp("Robot collided during move")
else
    disp("No collision detected")
end

lynx.stop() % Shut down ROS interface
