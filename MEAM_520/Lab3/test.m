% Test 1
map = loadmap('emptyMap.txt');
start = [0 0 0 0 0 0];
goal = [0 0 1.1 0 0 0];

isCollidedA = isRobotCollided(start,map);
isCollidedB = isRobotCollided(goal,map);
path = rrt(map, start, goal);

% Test 2
map = loadmap('map1.txt');
start = [0 0 -0.8 0.8 0 0];
goal = [0 0 1 0 0 0];
 
isCollidedA = isRobotCollided(start,map);
isCollidedB = isRobotCollided(goal,map);
path = rrt(map, start, goal);

% Test 3
map = loadmap('map2.txt');
start = [-1.2 0 0 0 0 0];
goal = [1.4 0 0 0 0 0];
 
isCollidedA = isRobotCollided(start,map);
isCollidedB = isRobotCollided(goal,map);
path = rrt(map, start, goal);

% Test 4
map = loadmap('map3.txt');
start = [0 0 0 0 0 0];
goal = [0 0 1 0 0 0];

isCollidedA = isRobotCollided(start,map);
isCollidedB = isRobotCollided(goal,map);
path = rrt(map, start, goal);

% Test 5
map = loadmap('map4.txt');
start = [1 0.5 0 0 0 0];
goal = [0 -1.2 0 0 0 0];

isCollidedA = isRobotCollided(start,map);
isCollidedB = isRobotCollided(goal,map);
path = rrt(map, start, goal);

% Test 6
map = loadmap('map5.txt');
start = [0 0 0 0 0 0];
goal = [0 -1.2 0 0.1 0 0];

isCollidedA = isRobotCollided(start,map);
isCollidedB = isRobotCollided(goal,map);
path = rrt(map, start, goal);
