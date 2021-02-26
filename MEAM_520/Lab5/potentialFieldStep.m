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

% Parameters
alpha = 0.01;
eta = 10^3;
rho = 200;
zeta = 0.1;
epsilon = 0.0075;

% Calculate final joint positions
[joint_pos_f,~] = calculateFK(qGoal);
p2f = joint_pos_f(2,:);
p3f = joint_pos_f(3,:);
p4f = joint_pos_f(4,:);
p5f = joint_pos_f(5,:);
p6f = joint_pos_f(6,:);

% Calculate current joint positions
[joint_pos,~] = calculateFK(qCurr);
p2 = joint_pos(2,:);
p3 = joint_pos(3,:);
p4 = joint_pos(4,:);
p5 = joint_pos(5,:);
p6 = joint_pos(6,:);

% Calculate attractive forces
Fa2 = -zeta*(p2-p2f)';
Fa3 = -zeta*(p3-p3f)';
Fa4 = -zeta*(p4-p4f)';
Fa5 = -zeta*(p5-p5f)';
Fa6 = -zeta*(p6-p6f)';

% Calculate repulsive forces
Fr2 = [0,0,0]';
Fr3 = [0,0,0]';
Fr4 = [0,0,0]';
Fr5 = [0,0,0]';
Fr6 = [0,0,0]';

A = size(map.obstacles,1);
for i=1:A
    [dist2, unit2] = distPointToBox(p2, map.obstacles(i,:));
    [dist3, unit3] = distPointToBox(p3, map.obstacles(i,:));
    [dist4, unit4] = distPointToBox(p4, map.obstacles(i,:));
    [dist5, unit5] = distPointToBox(p5, map.obstacles(i,:));
    [dist6, unit6] = distPointToBox(p6, map.obstacles(i,:));
    
    if dist2 < rho
        Fr2 = Fr2-eta*(1/dist2 - 1/rho)*(1/dist2^2)*unit2';
    end
    if dist3 < rho
        Fr3 = Fr3-eta*(1/dist3 - 1/rho)*(1/dist3^2)*unit3';
    end
    if dist4 < rho
        Fr4 = Fr4-eta*(1/dist4 - 1/rho)*(1/dist4^2)*unit4';
    end
    if dist5 < rho
        Fr5 = Fr5-eta*(1/dist5 - 1/rho)*(1/dist5^2)*unit5';
    end
    if dist6 < rho
        Fr6 = Fr6-eta*(1/dist6 - 1/rho)*(1/dist6^2)*unit6';
    end
end

% Calculate joint forces
F2 = Fa2 + Fr2;
F3 = Fa3 + Fr3;
F4 = Fa4 + Fr4;
F5 = Fa5 + Fr5;
F6 = Fa6 + Fr6;

% Convert joint forces to joint torques
J2 = calcJacobian(qCurr,2);
Jv2 = J2(1:3,:);

J3 = calcJacobian(qCurr,3);
Jv3 = J3(1:3,:);

J4 = calcJacobian(qCurr,4);
Jv4 = J4(1:3,:);

J5 = calcJacobian(qCurr,5);
Jv5 = J5(1:3,:);

J6 = calcJacobian(qCurr,6);
Jv6 = J6(1:3,:);

tau2 = vertcat(Jv2'*F2,[0;0;0;0]);
tau3 = vertcat(Jv3'*F3,[0;0;0]);
tau4 = vertcat(Jv4'*F4,[0;0]);
tau5 = [Jv5'*F5;0];
tau6 = Jv6'*F6;
tau_total = tau2 + tau3 + tau4 + tau5 + tau6;

% Update joint configuration 
qNext = qCurr;
for i = 1:4
    qNext(i) = qNext(i) + alpha * tau_total(i) / norm(tau_total);
end

if norm(qGoal - qNext) < epsilon
    isDone = true;
else
    isDone = false; 
end

end
