function [qNext, isDone] = potentialFieldStep(qCurr, map, qGoal)
% function [qNext, isDone] = potentialFieldStep(qCurr, map, qGoal)
% This function exectures one step of the potential field planner.
%
% INPUTS:
%   qCurr:   - 1x6 vector of the robots current configuration
%   map:     - the map object to plan in
%   qGoal:   - 1x6 vector of the goal configuration
%
% OUTPUTS:
%   qNext - 1x6 vector of the robots next configuration
%   isDone - a boolean that is true when the robot has reached the goal or
%            is stuck. false otherwise

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Number of obstacles
num_obstacles = size(map.obstacles, 1);

%Calculating current and final joint positions
[joint_position_curr,~] = calculateFK(qCurr);
[joint_position_goal,~] = calculateFK(qGoal);

%Setting the potential field parameters
%distance from goal to switch from conical to parabllic well
rho_a  = 100*ones(6,1);
%attractive field strength
zeta = 10000*ones(6,3);

%distance from obstacle to apply repulsice force
rho_obs = 30*ones(6,1);
%repulsive field strength
eta = 1e6*ones(6,3);


%Calculating  Attractive for for each joint
f_att = [];
 for i= 1:6
     distance_from_goal= norm(joint_position_curr(i,:)-joint_position_goal(i,:));
     
     
   	if distance_from_goal < rho_a(i)
     %Parabollic Well
     f_att(i,:) = -zeta(i)*(joint_position_curr(i,:)-joint_position_goal(i,:));
    else
     %Conical Well
     f_att(i,:)= -rho_a(i)*zeta(i)*((joint_position_curr(i,:)-joint_position_goal(i,:))/distance_from_goal);
    end
 end
 
 
%Calculating Repulsive forces for each joint for each obstacle
f_rep_all = zeros(6,3);
f_rep = zeros(6,3);

for j=1:num_obstacles
    obs = map.obstacles(j,:);  
    
    %Calculaing distance from the obstacle
    [rho_, unit_] = distPointToBox(joint_position_curr, obs);
    %scaling diatnce to appply padding around obstacle
    rho_ = rho_.* 0.9;
    
    for i = 1:6
        if rho_(i) > rho_obs(i);
            f_rep(i, :) = zeros(1, 3);
        else
            f_rep(i, :) = eta(i)*(1/rho_(i)-1/rho_obs(i))*(1/rho_(i).^2)* unit_(i, :);
        end
    end
    
    f_rep_all = f_rep + f_rep_all;
    
end

%Calculating the sum of all forces
f_total = (f_att + f_rep_all);
tau = zeros(6,1);

%Calculating Joint-Space efforts
for i = 2:4 
        J_ = calcJacobian(joint_position_curr, i);
    Jv = J_(1:3,:);
    i;
    Jv(:,end+1:6) = 0;
    tau = tau + Jv'*f_total(i,:)';
end

%setting the step step for next q value
step_size = 0.08*ones(1,6);

%Calcualting the next value for q
qNext = qCurr + step_size.*((tau/norm(tau))');

%Limiting the q values o within limits
upperLimit = [1.4000 1.4000 1.7000 1.7000 1.5000 30];
lowerLimit  = [-1.4000 -1.2000 -1.8000 -1.9000 -2 -15];

qNext(qNext>=upperLimit) = upperLimit(qNext>=upperLimit);
qNext(qNext<=lowerLimit) = lowerLimit(qNext<=lowerLimit);

%Setting gripper to be = qGoal
qNext(6) = qGoal(6);

%Checking of the q is close to qGoal
epsilon = 0.1*ones(1,6);

if abs(qGoal-qNext) < epsilon
    isDone = true;
else
    isDone = false;
end


end
    
    
    

