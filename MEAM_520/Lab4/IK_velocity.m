function [dq] = IK_velocity(q, v, omega, joint)
% function [dq] = IK_velocity(q, v, omega, joint)
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   q     - 1 x 6 vector corresponding to the robot's current configuration 
%   v     - The desired linear velocity in the world frame. If any element
%           is Nan, then that velocity can be anything
%   omega - The desired angular velocity in the world frame.
%           If any element is Nan, then that velocity can be anything
%   joint - an integer in [0,6] corresponding to which joint you are
%           tracking
%
% OUTPUT:
%   dq - 1 x 6 vector coresponding to the joint velocities. If v and omega
%        are infeasible, then dq should minimize the least squares error.
%        
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J = calcJacobian(q,joint);
velocity = vertcat(v,omega);

if sum(isnan(velocity)) > 0
    J(isnan(velocity)==1,:) = [];
    velocity(isnan(velocity)==1) = [];
end

J_pinv = pinv(J);
dq = (J_pinv*velocity)';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end