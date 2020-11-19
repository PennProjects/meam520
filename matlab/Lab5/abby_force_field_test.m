clc
clear
% 
addpath('maps')
map = loadmap('map2.txt');

num_obstacles = size(map.obstacles, 1)

qCurr = [0 0 0 0 0 0];
[joint_position_curr,~] = calculateFK(qCurr)

qGoal = [pi/4 -pi/4 0 pi/4 0 0]; 
[joint_position_goal,~] = calculateFK(qGoal)

rho_a  = 5*ones(6,1);
% obd
zeta = [1;1;1;1;1;1]*ones(1,3);

% repulsive force rho
rho_b = 5*ones(6,1);
eta = [10;10;10;10;10;10]*ones(1,3);

% (joint_position_curr(:,1)-joint_position_goal(:,1)).^2
a = (joint_position_curr(:,1)-joint_position_goal(:,1)).^2 ;
b = (joint_position_curr(:,2)-joint_position_goal(:,2)).^2 ;
c = (joint_position_curr(:,3)-joint_position_goal(:,3)).^2;

distance_from_goal = sqrt(a + b + c)

f_att = [];
f_rep = [];


 for i= 1:6
     
    % attractive force
    x = distance_from_goal <= rho_a
    f_att(x,:) = -zeta(x).*(joint_position_curr(x,:)-joint_position_goal(x,:));
    
    y = distance_from_goal > rho_a
    f_att(y,:)= -rho_a(y).*zeta(y).*((joint_position_curr(y,:)-joint_position_goal(y,:))/norm(joint_position_curr(y,:)-joint_position_goal(y,:)));
    
    
 end
 
% repulsive force
f_rep_all = [];

for j=1:num_obstacles
%     rho_i = [];
%     unit_i = [];
    obs = map.obstacles(j,:)
    rho_obs =  55*ones(6,1);    
   
    [rho_, unit_] = distPointToBox(joint_position_curr, obs)
    unit_
    rho_ = rho_.* 0.9
    
    y_r = rho_ > rho_obs
    f_rep(y_r, :) = zeros(sum(y_r), 3)
    
    x_r = rho_ <= rho_obs
    
    %f_rep_2(x_r, :) = eta(x_r).*(1/rho_(x_r)-1/rho_obs(x_r)).*(1/rho_(x_r).^2).* unit_(x_r, :)
    one = eta(x_r) %(3,1)
    two = (1/rho_(x_r))' %(1,3)
    three = (1/rho_obs(x_r))' %(1,3)
    four = (1/rho_(x_r).^2)' %(1,3)
    five= unit_(x_r, :) %(3,3)
    
    f_rep(x_r, :) = one.*(two-three).*four.*five
    
    f_rep_all = 
    
end


% F
% F = f_att + f_rep;
% 
% % J
% Jv = []; 
% for i:1=6
%     J_ = calcJacobian(joint_position_curr, i);
%     Jv = [Jv; ];
% end

 