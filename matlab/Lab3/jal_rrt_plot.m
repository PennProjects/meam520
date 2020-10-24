function [path] = jal_rrt_plot(map, start, goal)
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
robot.d5 = 68 + 12.5;
robot.lg = 35;
robot.lowerLim = [-1.4000 -1.2000 -1.8000 -1.9000 -2 -15];
robot.upperLim = [1.4000 1.4000 1.7000 1.7000 1.5000 30];
robot.d4 = 34;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

total_iterations = 3;

tree(1).coord = start; 
tree(1).parent = -1;

%%%%%%%%%
tree_end(1).coord = goal;
tree_end(1).parent = -1; 
%%%%%%%%%


epsilon = 0.01;
% reachedGoal = checkReachedGoal(start, goal, epsilon);
alpha = 1000;

%%%%
    num_added_tree = 0; 
%%%%

while num_added_tree ~= 2
    
    %%%%
    num_added_tree = 0; 
    %%%%
    
    % assume we have a new point
    temp_lowerlimit = min(start, goal)-epsilon;
    temp_upperlimit = max(start,goal)+epsilon;
    
    random_array = rand(1,6);
    
    random_angles = robot.lowerLim +  (robot.upperLim-robot.lowerLim).*random_array;
%     random_angles = temp_lowerlimit + (temp_upperlimit-temp_lowerlimit).*random_array;
    
    %keeping rand angles within limit
    %%%%%%%%%%
    random_angles(random_angles(:) < robot.lowerLim(:)) = robot.lowerLim(random_angles(:) < robot.lowerLim(:));
    random_angles(random_angles(:) > robot.upperLim(:)) = robot.upperLim(random_angles(:) > robot.upperLim(:));  
    %%%%%%%%%%
    
    
%     random_angles = robot.lowerLim + (robot.upperLim-robot.lowerLim).*random_array; 
    q_new = [random_angles(1) random_angles(2) random_angles(3) random_angles(4) start(5) start(6)]
%     q_new = [rand(1) rand(1) 0 0 0 0]
    % check whether this random point collides with obstacle
    %isCollided = checkCollision(q_new, map); 
      
%     if isCollided
%         continue;
%     end
%     
    % find q_a that is closest node in T_start
    dist_array = [];
    size_tree = size(tree, 2);
    for idx = 1:size_tree
        dist_array = [dist_array; sqrt(sum((q_new - tree(idx).coord).^2))];
    end
    [~, min_dist_idx] = min(dist_array);
    
    
    
    closest_q = tree(min_dist_idx).coord;
    
%     [start_points, end_points] = collision_check_points(q_new, closest_q); 
%     [isCollided] = checkCollision(start_points, end_points, map); 
    
    % check collision for points inbetween closest_q, q_new 
    % (interpolation in configuration space) 
    [points_in_between_start] = generate_points_inbetween(closest_q, q_new, alpha);
    
    all_between_robot_joint_start_start = [];
    all_between_robot_joint_end_start = []; 
    for num = 1:size(points_in_between_start, 1)
        [each_point_robot_joint_start_start, each_point_robot_joints_end_start ] = collision_check_points(points_in_between_start(num, :)); 
        all_between_robot_joint_start_start = [all_between_robot_joint_start_start;each_point_robot_joint_start_start];
        all_between_robot_joint_end_start = [all_between_robot_joint_end_start;each_point_robot_joints_end_start];

    end
    
    [isCollided2_start] = checkCollision(all_between_robot_joint_start_start, all_between_robot_joint_end_start, map); 
    
%     if isCollided 
%         continue
%     elseif isCollided2
%         continue
%     else
    if any(isCollided2_start)==0
        tree(size_tree+1).parent = min_dist_idx;
        tree(size_tree+1).coord = q_new;
        %%%
        num_added_tree = num_added_tree + 1;
        size_tree = size(tree, 2);
        %%%
    end
    
    
    
    %%%%%%% checking for tree_end 
    dist_array_end = [];
    size_tree_end = size(tree_end, 2);
    for idx = 1:size_tree_end
        dist_array_end = [dist_array_end; sqrt(sum((q_new - tree_end(idx).coord).^2))];
    end
    [~, min_dist_idx_end] = min(dist_array_end);
    
    closest_q_end = tree_end(min_dist_idx_end).coord;
    
%     [start_points_end, end_points_end] = collision_check_points(q_new, closest_q_end); 
%     [isCollided_end] = checkCollision(start_points_end, end_points_end, map); 
    
    % check collision for points inbetween closest_q, q_new 
    % (interpolation in configuration space) 
    [points_in_between_goal] = generate_points_inbetween(closest_q_end, q_new, alpha);
    
    all_between_robot_joint_start_goal = [];
    all_between_robot_joint_end_goal = []; 
    for num = 1:size(points_in_between_goal, 1)
        [each_point_robot_joint_start_goal, each_point_robot_joints_end_goal ] = collision_check_points(points_in_between_goal(num, :)); 
        all_between_robot_joint_start_goal = [all_between_robot_joint_start_goal;each_point_robot_joint_start_goal];
        all_between_robot_joint_end_goal = [all_between_robot_joint_end_goal;each_point_robot_joints_end_goal];

    end
    
    [isCollided2_end] = checkCollision(all_between_robot_joint_start_goal, all_between_robot_joint_end_goal, map); 
    
%     if isCollided_end 
%         continue
%     elseif isCollided2_end
%         continue
%     else
    if any(isCollided2_end)==0
        tree_end(size_tree_end+1).parent = min_dist_idx_end;
        tree_end(size_tree_end+1).coord = q_new;
        num_added_tree = num_added_tree + 1;
        size_tree_end = size(tree_end, 2);
    end
    
    %%%%%%%
        
    %For live tree plot
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    branch = [tree(min_dist_idx).coord; tree(size_tree).coord];
    plot3(start(1), start(2), start(3), '.', 'MarkerSize',50, 'color', 'b')
    hold on
    drawnow
    plot3(goal(1), goal(2), goal(3),'.', 'MarkerSize',50, 'color', '#B80F0A')
    plot3(branch(:,1), branch(:,2), branch(:,3) ,'o-', 'color', '#B80F0A')
    
    branch_end = [tree_end(min_dist_idx_end).coord; tree_end(size_tree_end).coord];
    plot3(branch_end(:,1), branch_end(:,2), branch_end(:,3) ,'o-', 'color', '#B80F0A')
    
    
    
    % Plot view configuration
    grid on
    title('Lynx robot configitarion space','FontSize', 30, 'FontWeight', 'bold')
    xlabel('Theta 1', 'FontSize', 20, 'FontWeight', 'bold')
    ylabel('Theta 2', 'FontSize', 20, 'FontWeight', 'bold')
    zlabel('Theta 3', 'FontSize', 20, 'FontWeight', 'bold')
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %reachedGoal = checkReachedGoal(q_new, goal, epsilon);
    if num_added_tree == 2
        path_from_start = [];
        path_from_goal = []; 
        
        %path = [goal];
        tree_node_num = size_tree;
        while(tree_node_num ~= -1)
            path_from_start = [tree(tree_node_num).coord; path_from_start];
            tree_node_num = tree(tree_node_num).parent;
        end
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
       
        path = [path_from_start; path_from_goal];
        
        %For live tree plot
         %%%%%%%%%%%%%%%%%%%%%%
         plot3(start(1), start(2), start(3), '.', 'MarkerSize',50, 'color', '#2E8B57')
          plot3(goal(1), goal(2), goal(3),'.', 'MarkerSize',50, 'color', '#2E8B57')
         plot3(path(:,1), path(:,2), path(:,3),'o-', 'LineWidth', 3,'color','#2E8B57')
         %%%%%%%%%%%%%%%%%%%%%%%%
        break
    end
    
    
    
    
%     
%     for j  = 1:size(tree
%         tmp_dist =  sqrt((q_new(1) - tree(j).coord(1))^2 + (q_new(2) - tree(j).coord(2))^2 + (q_new(3) - tree(j).coord(3))^2);
%         if closest_dist < tmp_dist
%             closest_dist = tmp_dist; 
%             closest_node = j;
%             q_a = tree(j).q;
%         end
%     end
    
    % or move towards that q_new position by some delta and set it q_a_dash
    
    
    % if not collide(q, q_a') -> add (q, q_a') to T_start
    
    
%     if isCollided == 0
%         tree(i).coord = q_new;
%         tree(i).parent = closest_q;
%     end
    
   
end

%path = [pi/4 0 0 0 0 0; -pi/4 0 0 0 0 0; path];
% % path = [pi/4 pi/4 pi/4 pi/4 pi/4 pi/4; 
% %         -pi/4 -pi/4 pi/4 pi/4 pi/4 pi/4;]
% path = [start; path];
% path = [path; goal]; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % find n points in between
      function [points_in_between] = generate_points_inbetween(p1, p2, alpha)
        % distance between 2 points
        points_in_between = [];
        dist = norm(p2-p1); 
%         normalized_dist = dist / sqrt(numel(p2));
        normalized_dist = norm(start-goal);        
        ratio = dist / normalized_dist;
        num_points = round(alpha * ratio);
        if(num_points==0)
            num_points  = 1;
        end
        
        
        for i = 0:num_points
            tmp = p1 + i*(p2 - p1)/num_points;
            points_in_between = [points_in_between; tmp]; 
        end
    end
    
    
    
%     function [start_points, end_points] = collision_check_points(p1,p2)
%         [jointPositions_p1,~] = calculateFK(p1);
%         [jointPositions_p2,~] = calculateFK(p2);
% 
% 
%         robot_at_p1 = [jointPositions_p1(1:5,:), jointPositions_p1(2:6,:)];
%         robot_at_p2 = [jointPositions_p2(1:5,:), jointPositions_p2(2:6,:)];
% 
%         concat_points = [robot_at_p1;robot_at_p2];
% 
%         start_points = [concat_points(:,1:3)];
%         end_points = [concat_points(:,4:6)];
% 
%     end

    function [start_points, end_points] = collision_check_points(p1)
        [jointPositions_p1,~] = calculateFK(p1);
    
        robot_at_p1 = [jointPositions_p1(1:5,:), jointPositions_p1(2:6,:)];

        start_points = [robot_at_p1(:,1:3)];
        end_points = [robot_at_p1(:,4:6)];

    end

%     function [reachedGoal] = checkReachedGoal(q_new, goal, epsilon)
%         if ((abs(q_new(1) - goal(1)) <= epsilon) && (abs(q_new(2) - goal(2)) <= epsilon) && (abs(q_new(3) - goal(3)) <= epsilon)) 
%             reachedGoal = 1;
%         else
%             reachedGoal = 0;
%         end
%     end

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
