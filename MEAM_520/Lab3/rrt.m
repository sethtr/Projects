function [path] = rrt(map, start, goal)
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

path = zeros(0,6);
lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15];
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; 

%% Identify valid configurations

A = 400; % number of sampled configurations
B = 30; % number of neighboring configurations
C = 15; % nubmer of interpolations between configurations
D = 0; % number of valid configurations

cspace_nodes = zeros(0,6);
for i = 1:A 
    q = random('uniform', lowerLim(1:6), upperLim(1:6)); 
    if ~isRobotCollided(q, map)
        D = D + 1;
        cspace_nodes(D,:) = q;
    end
end

%% Build RRT
G = graph;
G = addnode(G,D);

for i = 1:D
    distances = max(abs(cspace_nodes - ...
        cspace_nodes(i,:)),[],2);
    [~,I] = sort(distances);% sort distances between configurations 
    
    for j = 2:(B + 1) % identify B nearest neighboring configurations
        branch_collision = false;
        ranking = I(j);
        interpolation = cspace_nodes(i,:) + (0:1/C:1)' * ...
            (cspace_nodes(ranking,:) - cspace_nodes(i,:));
        for k = 2:(C-1)
            q = interpolation(k,:);
            collision = isRobotCollided(q,map); % check for collisions
            branch_collision = branch_collision || collision;
        end
        if ~branch_collision && ~ismember(ranking,neighbors(G,i)) &&...
                i ~= ranking
            heuristic = norm(cspace_nodes(i,:) - ...
                cspace_nodes(ranking,:),Inf); % assign heuristic to branch
            G = addedge(G,i,ranking,heuristic); % add branch to graph
        end
    end
end
%% Find path

% Find closest node to start configuration i.e. nodeA
A_distances = max(abs(cspace_nodes - start),[],2);
[~,A_I] = sort(A_distances);

indexA = 0;
distanceA = 0;
nodeA = zeros(1,6);

for i = 1:length(A_I)
    branch_collision = false;
    ranking = A_I(i);
    interpolation = start + (0:1/C:1)' * ...
        (cspace_nodes(ranking,:) - start);
    for k = 1:C
        q = interpolation(k,:);
        collision = isRobotCollided(q,map); % check for collisions
        branch_collision = branch_collision || collision;
    end
    if ~branch_collision
        indexA = ranking;
        distanceA = A_distances(ranking);
        nodeA = cspace_nodes(ranking,:);
        break
    end
end

% Find closest node to goal configuration i.e. nodeB
B_distances = max(abs(cspace_nodes - goal),[],2);
[~,B_I] = sort(B_distances);

indexB = 0;
distanceB = 0;
nodeB = zeros(1,6);

for i = 1:length(B_I)
    branch_collision = false;
    ranking = B_I(i);
    interpolation = goal + (0:1/C:1)' * ...
        (cspace_nodes(ranking,:) - goal);
    for k = 1:C  
        q = interpolation(k,:);
        collision = isRobotCollided(q,map); % check for collisions
        branch_collision = branch_collision || collision;
    end
    if ~branch_collision
        indexB = ranking;
        distanceB = B_distances(ranking);
        nodeB = cspace_nodes(ranking,:);
        break
    end
end

% Build path between start and nodeA
stepsA = round(distanceA * 15); 
interpolationA = start + (0:1/stepsA:1)' * (nodeA - start);
path = vertcat(path,interpolationA);

% Build path between nodeA and nodeB w/ Dijkstra's algorithm
P = shortestpath(G,indexA,indexB,'Method','positive');
for i = 1:(length(P) - 1)
    j = P(i);
    k = P(i+1);
    distanceAB = norm(cspace_nodes(k,:) - cspace_nodes(j,:),Inf);
    stepsAB = round(distanceAB * 15);
    interpolation = cspace_nodes(j,:) + (0:1/stepsAB:1)' * ...
        (cspace_nodes(k,:) - cspace_nodes(j,:));
    path = vertcat(path,interpolation);
end

% Build path between nodeB and goal
stepsB = round(distanceB * 15);
interpolationB = nodeB + (0:1/stepsB:1)' * (goal - nodeB);
path = vertcat(path,interpolationB);

end
