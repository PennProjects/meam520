%% Setup
clear all
close all
addpath('maps')
addpath('../Core')

%% Simulation Parameters

start = [0,0,0,0,0,0]; 
goal = [-1.4,pi/4,0,0,0,0];

map = loadmap('map2.txt');

% Find collision-free path using RRT to get list of waypoints
[path] = jal_rrt_plot(map, start, goal);

% [path] = astar(map, start, goal);

%start ROS
lynx = ArmController();
pause(1) % wait for setup
collision = false;
lynx.set_pos([0 0 0 0 0 0])
pause(10)
path
% iterate over target waypoints
for target_index = 1:length(path(:,1))
    q = path(target_index, :);
    disp("Goal:")
    disp(q)
    lynx.set_pos(q)
    reached_target = false;
    e = 0.001;
    % Define relevant variables here:
    count = 0;
    while ~reached_target
        % Check if robot is collided then wait
%         collision = collision || lynx.is_collided();
        pause(0.1)
        collision = collision | lynx.is_collided();

        % Add Student code here to decide if controller should send next
        % target or continue to wait. Do NOT add additional pauses to control
        % loop. You will likely want to use lynx.get_state() to decide when to
        % move to the next target.
        count = count + 1;
        if count > 20
            reached_target = true;
            count = 0;
        end
% 
%           [pos, vel] = lynx.get_state();
%           if abs(pos(1)- q(1)) <=e && abs(pos(2)-q(2)) <= e && abs(pos(3)-q(3))<=e
%               reached_target = true;
%           else
%               lynx.set_pos(q)
%           end

        % End of student code
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
