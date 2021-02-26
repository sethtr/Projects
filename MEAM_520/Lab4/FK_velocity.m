function [v omega] = FK_velocity(q, dq, joint)
% function [v omega] = FK_velocity(q dq joint)
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   q  - 1 x 6 vector corresponding to the robot's current configuration
%   dq - 1 x 6 vector corresponding to the robot's current joint velocities
%   joint - an integer in [0,6] corresponding to which joint you are
%           tracking
%
% OUTPUT:
%   v     - The resulting linear velocity in the world frame
%   omega - The resulting angular velocity in the world frame
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J = calcJacobian(q,joint);
velocity = J*dq';
v = velocity(1:3);
omega = velocity(4:6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end