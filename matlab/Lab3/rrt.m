function [path] = rrt_abby(map, start, goal)
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

total_iterations = 3;

tree(1).coord = start; 
tree(1).parent = 0;

for i = 2:total_iterations
    % assume we have a new point
    q_new = [robot.lowerLim(1)*rand(1) robot.lowerLim(2)*rand(1) robot.lowerLim(3)*rand(1) start(4) start(5) start(6)];
    
    % check whether this random point collides with obstacle
    isCollided = checkCollision(q_new); 
      
    if isCollided
        continue;
    end
    
    % find q_a that is closest node in T_start
    closest_dist = Inf;
    for j  = 1:i
        tmp_dist =  sqrt((curr_q(1) - tree(j).q(1))^2 + (curr_q(1) - tree(j).q(2))^2 + (curr_q(1) - tree(j).q(3))^2);
        if closest_dist < tmp_dist
            closest_dist = tmp_dist; 
            closest_node = j;
            q_closest = tree(j).q;
        end
    end
    
    % check collision between q_new and q_closest 
    
    % if not collide(q, q_a') -> add (q, q_a') to T_start
    
    
    if isCollided == 0
        tree(i).coord = q_new;
        tree(i).parent = closest_node;
    end
    
    reachedGoal = checkReachedGoal(q_new, goal);
    if reachedGoal
        path = [];
        tree_node_num = i;
        while(tree_node_num ~= -1)
            path = [path; tree(tree_node_num)];
            tree_node_num = tree_node(tree_node_num).parent;
        end
        break
    end
    
end

% path = [pi/4 0 0 0 0 0; -pi/4 0 0 0 0 0];
% % path = [pi/4 pi/4 pi/4 pi/4 pi/4 pi/4; 
% %         -pi/4 -pi/4 pi/4 pi/4 pi/4 pi/4;]
% path = [start; path];
% path = [path; goal]; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [reachedGoal] = checkReachedGoal(q_new, goal)
        if q_new(1) == goal(1) && q_new(2) == goal(2) && q_new(3) == goal(3) 
            reachedGoal = 1;
        else
            reachedGoal = 0;
        end
    end
    

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
