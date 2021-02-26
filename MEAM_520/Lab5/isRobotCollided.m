function [isCollided] = isRobotCollided(q, map)

A = size(map.obstacles,1);
[jointPositions,T0e] = calculateFK(q);
isCollided = false;

for i = 1:A
    for j = 1:5
    link_collision = detectCollision(jointPositions(j,:),...
        jointPositions(j+1,:),map.obstacles(i,:));
    isCollided = isCollided || link_collision;
    end
    end_effector_collision = detectCollision(T0e(1:3,4)',...
        jointPositions(6,:),map.obstacles(i,:));
    isCollided = isCollided || end_effector_collision;
end

end
