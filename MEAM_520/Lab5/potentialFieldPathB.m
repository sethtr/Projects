function [path] = potentialFieldPathB(map, qStart, qGoal)
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

path = zeros(0,6);
path = vertcat(path,qStart);
qCurr = qStart;
isDone = false;
while ~isDone
    [qNext, isDone] = potentialFieldStep(qCurr,map,qGoal);
    path = vertcat(path,qNext);
    qCurr = qNext;
end

end