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

path = zeros(0,6);
path = vertcat(path,qStart);

qCurr = qStart;
isDone = false;

while ~isDone
    [qNext, isDone] = potentialFieldStep(qCurr, map, qGoal);
    
    % Tolerance to local minimum
    epsilonm = 0.01;
    E = zeros(0,10);
    if (size(path,1) >= 10)
        stuck = true;
        for ii = 1:10
            E(ii) = (norm(qNext-path(end-(ii-1),:)));
            stuck = stuck && E(ii) < epsilonm;
        end
        if stuck
            isDone = true;
            disp("Local Minimal")
        else
            path = vertcat(path, qNext);
            qCurr = qNext;
        end
    else
        path = vertcat(path, qNext);
        qCurr = qNext;
    end
end

end