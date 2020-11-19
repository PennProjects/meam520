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


rho_a  = [2 2 2 2 2 2];
zeta = [1 1 1 1 1 1];

distance_from_goal = abs(qCurr-qGoal)

f_att = [];



% attractive force
para_well = distance_from_goal <= rho_a
f_att(para_well) = -zeta(para_well).*(qCurr(para_well)-qGoal(para_well))

con_well = distance_from_goal > rho_a
f_att(con_well)= -rho_a(con_well).*zeta(con_well).*((qCurr(con_well)-qGoal(con_well))/norm(qCurr(con_well)-qGoal(con_well)))

% repulsive force
