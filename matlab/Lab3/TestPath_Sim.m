%% Setup
clear all
close all
addpath('maps')
addpath('../Core')

%% Simulation Parameters

start = [0,0,0,0,0,0];
goal = [0,0,1.1,0,0,0];

map = loadmap('map1.txt');

% Find collision-free path using RRT to get list of waypoints
[path] = rrt(map, start, goal);

%[path] = astar(map, start, goal);

%start ROS
lynx = ArmController();
pause(1) % wait for setup
collision = false;
lynx.set_pos([0 0 0 0 0 0])
pause(10)

% iterate over target waypoints
for target_index = 1:length(path(:,1))
    q = path(target_index, :);
    disp("Goal:")
    disp(q)
    lynx.set_pos(q)
    reached_target = false;

    % Define relevant variables here:

    while ~reached_target
        % Check if robot is collided then wait
        pause(0.1)
        collision = collision || lynx.is_collided();
       
        % Add Student code here to decide if controller should send next
        % target or continue to wait. Do NOT add additional pauses to control
        % loop. You will likely want to use lynx.get_state() to decide when to
        % move to the next target.
        [pos, vel] = lynx.get_state()
         if pos == q
             reached_target = true;
         else
             lynx.set_pos(q)
         end
          %reached_target = true;
          
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
