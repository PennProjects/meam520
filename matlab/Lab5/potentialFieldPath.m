function [path] = potentialFieldPath(map, qStart, qGoal)
%function [path] = potentialFieldPath(map, qStart, qGoal)
% This function plans a path through the map using a potential field
% planner
%
% INPUTS:
%   map      - the map object to plan in
%   qStart   - 1x6 vector of the starting configuration
%   qGoal:   - 1x6 vector of the goal configuration
%
% OUTPUTS:
%   path - Nx6 vector of the path from start to goal


qCurr = qStart;
isDone = false;
path = [];

while(~isDone)
[qNext, isDone] = potentialFieldStep(qCurr, map, qGoal)

if isnan(qNext)
    qCurr
    qNext
    break
end

path = [path; qNext];

qCurr = qNext;
end

path = [qStart;
        path;
        qGoal];

end