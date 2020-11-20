clc
clear
% 
addpath('maps')
map = loadmap('map2.txt');

num_obstacles = size(map.obstacles, 1);

qCurr = [0 0 0 0 0 0];
[joint_position_curr,~] = calculateFK(qCurr);

qGoal = [0,pi/4,pi/4,pi/4,0,0];
[joint_position_goal,~] = calculateFK(qGoal);

rho_a  = 1*ones(6,1);
% obd
zeta = [1;1;1;1;1;1]*ones(1,3);

% repulsive force rho
rho_b = 5*ones(6,1);
eta = [10;10;10;10;10;10]*ones(1,3);

% % (joint_position_curr(:,1)-joint_position_goal(:,1)).^2
% a = (joint_position_curr(:,1)-joint_position_goal(:,1)).^2 ;
% b = (joint_position_curr(:,2)-joint_position_goal(:,2)).^2 ;
% c = (joint_position_curr(:,3)-joint_position_goal(:,3)).^2;
% 
% distance_from_goal = sqrt(a + b + c);

f_att = [];
f_rep = [];


 for i= 1:6
     
     distance_from_goal= norm(joint_position_curr(i,:)-joint_position_goal(i,:));
%      i
%      (joint_position_curr(i,:)-joint_position_goal(i,:))
%      (joint_position_curr(i,:)-joint_position_goal(i,:))/distance_from_goal
%     distance_from_goal = sqrt(sum((joint_position_curr(i,:)- joint_position_goal(i,:)).^2)) 
    
    % attractive force
   	if distance_from_goal < rho_a(i)
     f_att(i,:) = -zeta(i)*(joint_position_curr(i,:)-joint_position_goal(i,:));
    else
%      f_att(i,:)= -rho_a(i).*zeta(i).*((joint_position_curr(i,:)-joint_position_goal(i,:))/distance_from_goal;
     f_att(i,:)= -rho_a(i)*zeta(i)*((joint_position_curr(i,:)-joint_position_goal(i,:))/distance_from_goal);
    end
 end
 
% repulsive force
f_rep_all = zeros(6,3);

for j=1:num_obstacles
%     rho_i = [];
%     unit_i = [];
    obs = map.obstacles(j,:);
    rho_obs =  5*ones(6,1);    
   
    [rho_, unit_] = distPointToBox(joint_position_curr, obs);
%     unit_;
    rho_ = rho_.* 0.9;
    
    for i = 1:6
        
        if rho_(i) > rho_obs(i);
            f_rep(i, :) = zeros(1, 3);
        else
            f_rep(i, :) = eta(i)*(1/rho_(i)-1/rho_obs(i))*(1/rho_(i).^2)* unit_(i, :)
        end
%     one = eta(x_r)'; %(3,1)
%     two = (1./rho_(x_r))';%(1,3);
%     three = (1./rho_obs(x_r))'; %(1,3);
%     four = (1./rho_(x_r).^2)'; %(1,3);
%     x = (one.*(two-three).*four)';
%     
%     five= unit_(x_r, :); %(3,3)
    end
%     f_rep(x_r, :) = x.*five;
    
    f_rep_all = f_rep + f_rep_all;
    
end

f_att 

% F
F = (f_att + f_rep_all)
tau = zeros(6,1);

for i = 1:6 
    J_ = calcJacobian(joint_position_curr, i);
    J_ = [[0;0;0;0;0;0],J_];
    Jv = J_(1:3,:);
    Jv(:,end+1:6) = 0;
    tau = tau + Jv'*F(i,:)';
% %     Jv = [Jv;J_ ];
end

step_size = 0.02*ones(1,6);

qNext = qCurr + step_size*tau/norm(tau);

epsilon = 0.1*ones(1,6);

% a = qNext-qGoal
if abs(qNext-qGoal) <epsilon
    isDone = true;
else
    isDone = false;
end



 