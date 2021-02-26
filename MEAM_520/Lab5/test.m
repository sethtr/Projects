map = loadmap('map1.txt');
qStart = [0 0 0 0 0 0];
qGoal = [1.3 0 0 0 0 0];

collisionA = isRobotCollided(qStart, map);
collisionB = isRobotCollided(qStart, map);
path = potentialFieldPath(map, qStart, qGoal);