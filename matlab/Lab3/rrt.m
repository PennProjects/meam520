function [path] = rrt(map, start, goal)
% RRT Find the shortest path from start to goal.
%   PATH = rrt(map, start, goal) returns an mx6 matrix, where each row
%   consists of the configuration of the Lynx at a point on the path. The
%   first row is start and the last row is goal. If no path is found, PATH
%   is a 0x6 matrix.
%
% INPUTS:
%   map     - the map object to plan in
%   start   - 1x6 vector of the starting configuration
%   goal:   - 1x6 vector of the goal configuration


%% Prep Code

path = [];

robot.d1 = 76.2;
robot.a2 = 146.05;
robot.a3 = 187.325;
robot.d5 = 68;
robot.lg = 35;
robot.lowerLim = [-1.4000 -1.2000 -1.8000 -1.9000 -2 -15];
robot.upperLim = [1.4000 1.4000 1.7000 1.7000 1.5000 30];
robot.d4 = 34;

q_init = robot.lowerLim;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

total_iterations = 3;

for i = 1:total_iterations
    % assume we have a new point
    q_new = [q_init(1)*rand(1) q_init(2)*rand(1) q_init(3)*rand(1) q_init(4)*rand(1) q_init(5)*rand(1) q_init(6)*rand(1)];
    
    isCollided = checkCollision(q_new); 
      
    if isCollided
        continue;
    end
    
    path = [path; q_new];
    q_init = q_new;
    
end

path = [pi/4 0 0 0 0 0; -pi/4 0 0 0 0 0];
% path = [pi/4 pi/4 pi/4 pi/4 pi/4 pi/4; 
%         -pi/4 -pi/4 pi/4 pi/4 pi/4 pi/4;]
path = [start; path];
path = [path; goal]; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function [isCollided] = checkCollision(q_new)
        % For each obstacle in the space
        mrgn = 25; 
        [jointPositions,T0e] = calculateFK(q_new); 
        linePt1 = jointPositions(1:5, :);
        linePt2 = jointPositions(2:6, :);
        
        for ii=1:size(map.obstacles)
            % Pad the obstacle
            currObstacle = [map.obstacles(ii,1:3)-mrgn,map.obstacles(ii,4:6)+mrgn];

            % Check for collisions with all the lines, which is a check for all
            % links of the robot.
            isCollided = detectCollision(linePt1, linePt2,currObstacle);
            
        end
    end


end
