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
% robot.d5 = 68;
%get frame centre at tip of gripper
robot.d5 = 68 + 12.5;
robot.lg = 35;
robot.lowerLim = [-1.4000 -1.2000 -1.8000 -1.9000 -2 -15];
robot.upperLim = [1.4000 1.4000 1.7000 1.7000 1.5000 30];
robot.d4 = 34;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% defining a struct 
%with coord as the point and parent as the index of parent point
% tree form start point
tree(1).coord = start; 
tree(1).parent = -1;

%tree form goal
tree_end(1).coord = goal;
tree_end(1).parent = -1; 

%max number of points to check for collision between 2 points
number_of_points_to_check= 1000;

%number of trees point is added to
num_added_tree = 0; 

% max iteration if no path is found
max_iterations = 100000;
iteration_count = 0;
 
while (iteration_count < max_iterations)
    
    %iteration counter
    iteration_count = iteration_count +1 ;    
 
    num_added_tree = 0; 
    
    %to find a randon point in configuration space    
    random_array = randi([0,1],[1,6]);
    
    %adding epsilon to angle 
    random_angles = robot.lowerLim +  (robot.upperLim-robot.lowerLim).*random_array;
    
    %selecting a new point
     q_new = [random_angles(1) random_angles(2) random_angles(3) random_angles(4) random_angles(5), start(6)];
    

      
    % finding a point in the tree from start
    %the distance between the point in consideration and every point in the tree is 
    %calculated and point with the minimum distance is found
    dist_array = [];
    size_tree = size(tree, 2);
    for idx = 1:size_tree
        dist_array = [dist_array; sqrt(sum((q_new - tree(idx).coord).^2))];
    end
    [~, min_dist_idx] = min(dist_array);
    
    closest_q = tree(min_dist_idx).coord;
    
    %generating points in between the q_new and closest point in tree to check for
    %collision along line joining q_new and closest point in tree 
    [points_in_between_start] = generate_points_inbetween(closest_q, q_new, number_of_points_to_check);
    
    all_between_robot_joint_start_start = [];
    all_between_robot_joint_end_start = []; 
    for num = 1:size(points_in_between_start, 1)
        [each_point_robot_joint_start_start, each_point_robot_joints_end_start ] = collision_check_points(points_in_between_start(num, :)); 
        all_between_robot_joint_start_start = [all_between_robot_joint_start_start;each_point_robot_joint_start_start];
        all_between_robot_joint_end_start = [all_between_robot_joint_end_start;each_point_robot_joints_end_start];
    end
    
    [isCollided2_start] = checkCollision(all_between_robot_joint_start_start, all_between_robot_joint_end_start, map); 
    
    % if q_new, and all points sampled in between q_new and closest_qare
    % collision free add q_new to tree and tag its parent as closest_q
    if any(isCollided2_start)==0
        tree(size_tree+1).parent = min_dist_idx;
        tree(size_tree+1).coord = q_new;
        num_added_tree = num_added_tree + 1;
        size_tree = size(tree, 2);
    end
    
    
    %%%%%%% Repeating the above process for tree from goal%%%%%%%%
    
    
    % finding a point in the tree from goal
    %the distance between the point in consideration and every point in the tree is 
    %calculated and point with the minimum distance is found
    dist_array_end = [];
    size_tree_end = size(tree_end, 2);
    for idx = 1:size_tree_end
        dist_array_end = [dist_array_end; sqrt(sum((q_new - tree_end(idx).coord).^2))];
    end
    [~, min_dist_idx_end] = min(dist_array_end);
    closest_q_goal = tree_end(min_dist_idx_end).coord;
    
    %generating points in between the q_new and closest point in tree to check for
    %collision along line joining q_new and closest point in tree 
    [points_in_between_goal] = generate_points_inbetween(closest_q_goal, q_new, number_of_points_to_check);
    
    all_between_robot_joint_start_goal = [];
    all_between_robot_joint_end_goal = []; 
    for num = 1:size(points_in_between_goal, 1)
        [each_point_robot_joint_start_goal, each_point_robot_joints_end_goal ] = collision_check_points(points_in_between_goal(num, :)); 
        all_between_robot_joint_start_goal = [all_between_robot_joint_start_goal;each_point_robot_joint_start_goal];
        all_between_robot_joint_end_goal = [all_between_robot_joint_end_goal;each_point_robot_joints_end_goal];

    end
    
    [isCollided2_end] = checkCollision(all_between_robot_joint_start_goal, all_between_robot_joint_end_goal, map); 
    
    
    % if q_new, and all points sampled in between q_new and closest_q_goal are
    % collision free add q_new to tree and tag its parent as closest_q
    if any(isCollided2_end)==0
        tree_end(size_tree_end+1).parent = min_dist_idx_end;
        tree_end(size_tree_end+1).coord = q_new;
        num_added_tree = num_added_tree + 1;
        size_tree_end = size(tree_end, 2);
    end
    
    %%%%%%%Points are now added to the tree%%%%%%%%%%
        
    %For live tree plot
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %plot start and goal
    plot3(start(1), start(2), start(3), '.', 'MarkerSize',50, 'color', '#0E4D92')
    text(start(1), start(2), start(3), 'Start', 'FontSize', 15)
    hold on
    drawnow
    plot3(goal(1), goal(2), goal(3),'.', 'MarkerSize',50, 'color', '#B80F0A')
    text(goal(1), goal(2), goal(3), 'Goal', 'FontSize', 15)
    
    %create a branch and plot from start tree
    branch = [tree(min_dist_idx).coord; tree(size_tree).coord];
    plot3(branch(:,1), branch(:,2), branch(:,3) ,'o-', 'color', '#0E4D92')
    
    %create branch and plot from goal tree
    branch_goal = [tree_end(min_dist_idx_end).coord; tree_end(size_tree_end).coord];
    plot3(branch_goal(:,1), branch_goal(:,2), branch_goal(:,3) ,'o-', 'color', '#B80F0A')
    
    % Plot view configuration
    grid on
    title(['Lynx robot configuration space at iterations :  ' num2str(iteration_count) ],'FontSize', 20, 'FontWeight', 'bold')
    xlabel('Theta 1', 'FontSize', 20, 'FontWeight', 'bold')
    ylabel('Theta 2', 'FontSize', 20, 'FontWeight', 'bold')
    zlabel('Theta 3', 'FontSize', 20, 'FontWeight', 'bold')
   
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Check if we have a path from start to goal
    if num_added_tree == 2
        path_from_start = [];
        path_from_goal = []; 
        
        %using the index of parent points build tree from start
        tree_node_num = size_tree
        while(tree_node_num ~= -1)
            path_from_start = [tree(tree_node_num).coord; path_from_start];
            tree_node_num = tree(tree_node_num).parent;
        end
        
        %using the index of parent points build tree from goal
        first = 0;
        tree_node_num_end = size_tree_end;
        while(tree_node_num_end ~= -1)
            if first == 0
                first = first + 1;
                tree_node_num_end = tree_end(tree_node_num_end).parent;
                continue
            end
            path_from_goal = [path_from_goal; tree_end(tree_node_num_end).coord];
            tree_node_num_end = tree_end(tree_node_num_end).parent;
        end
        
        %join the 2 trees
        path = [path_from_start; path_from_goal];
        
        %Plot final path
         %%%%%%%%%%%%%%%%%%%%%%
         plot3(start(1), start(2), start(3), '.', 'MarkerSize',50, 'color', '#2E8B57')
          plot3(goal(1), goal(2), goal(3),'.', 'MarkerSize',50, 'color', '#2E8B57')
         plot3(path(:,1), path(:,2), path(:,3),'o-', 'LineWidth', 3,'color','#2E8B57')
         %%%%%%%%%%%%%%%%%%%%%%%%
        break
    end
    
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % find n points in between 2 given points
      % here we scale the number of points to generate based on the
      % distance between start and end and siatance between the 2 points of
      % interest
      
      function [points_in_between] = generate_points_inbetween(p1, p2, number_of_points_to_check)
        % distance between 2 points
        points_in_between = [];
        dist = norm(p2-p1); 
        normalized_dist = norm(start-goal);        
        ratio = dist / normalized_dist;
        num_points = round(number_of_points_to_check * ratio);
        if(num_points==0)
            num_points  = 1;
        end        
        for i = 0:num_points
            tmp = p1 + i*(p2 - p1)/num_points;
            points_in_between = [points_in_between; tmp]; 
        end
      end
    
    % function to genrate the pairs of points that form the robot links at
    % any given configuration
    function [start_points, end_points] = collision_check_points(p1)
        [jointPositions_p1,~] = calculateFK(p1);
    
        robot_at_p1 = [jointPositions_p1(1:5,:), jointPositions_p1(2:6,:)];

        start_points = [robot_at_p1(:,1:3)];
        end_points = [robot_at_p1(:,4:6)];

    end

    %function to check for collision of robot with obstacle
    function [isCollided] = checkCollision(start_pts, end_pts, map)
        % For each obstacle in the space
        mrgn = 25; 
        
        for ii=1:size(map.obstacles)
            % Pad the obstacle
            currObstacle = [map.obstacles(ii,1:3)-mrgn,map.obstacles(ii,4:6)+mrgn];

            % Check for collisions with all the lines, which is a check for all
            % links of the robot.
            isCollided = detectCollision(start_pts, end_pts,currObstacle);
            
        end

    end


end