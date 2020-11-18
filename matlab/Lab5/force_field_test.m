

clc
clear
% 
% addpath('maps')
% map = loadmap('map2.txt');

% o = size(map.obstacles, 1)

qCurr = [0 0 0 0 0 0];
[joint_position_curr,~] = calculateFK(qCurr)

qGoal = [pi/4 -pi/4 0 pi/4 0 0]; 
[joint_position_goal,~] = calculateFK(qGoal)

rho_a  = 5*ones(6,1)
% obd
zeta = [1;1;1;1;1;1]*ones(1,3)



% (joint_position_curr(:,1)-joint_position_goal(:,1)).^2
a = (joint_position_curr(:,1)-joint_position_goal(:,1)).^2 ;
b = (joint_position_curr(:,2)-joint_position_goal(:,2)).^2 ;
c = (joint_position_curr(:,3)-joint_position_goal(:,3)).^2;

distance_from_goal = sqrt(a + b + c)

f_att = [];



% if(distance_from_goal <= rho_a)
%     
    y = distance_from_goal > rho_a
 for i= 1:6
     x = distance_from_goal <= rho_a
    f_att(x,:) = -zeta(x).*(joint_position_curr(x,:)-joint_position_goal(x,:))
    
    y = distance_from_goal > rho_a
    f_att(y,:)= -rho_a(y).*zeta(y).*((joint_position_curr(y,:)-joint_position_goal(y,:))/norm(joint_position_curr(y,:)-joint_position_goal(y,:)))
 end
