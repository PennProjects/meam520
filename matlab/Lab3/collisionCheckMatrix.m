

p1 = [0,0,0,0,0,0];
p2 = [pi/4, pi/4,-pi/4,0,0,0];

function [start_points, end_points] = collision_check_points[p1,p2];
[jointPositions_p1,~] = calculateFK(p1);
[jointPositions_p2,~] = calculateFK(p2);


robot_at_p1 = [jointPositions_p1(1:5,:), jointPositions_p1(2:6,:)];
robot_at_p2 = [jointPositions_p2(1:5,:), jointPositions_p2(2:6,:)];

concat_points = [robot_at_p1;robot_at_p2];

start_points = [concat_points(:,1:3)]
end_points = [concat_points(:,4:6)]

end







