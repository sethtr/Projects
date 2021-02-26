function [q isPos] = calculateIK_B(T0e)
% CALCULATEIK - Please rename this function using your group # in
%   both the function header and the file name. 
%
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   T - 4 x 4 homogeneous transformation matrix, representing 
%               the end effector frame expressed in the base (0) frame
%               (position in mm)
%
% OUTPUT:
%   q          - a n x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad) 
%                which are required for the Lynx robot to reach the given 
%                transformation matrix T. Each row represents a single
%                solution to the IK problem. If the transform is
%                infeasible, q should be all zeros.
%   isPos      - a boolean set to true if the provided
%                transformation T is achievable by the Lynx robot, ignoring
%                joint limits
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
d1 = 76.2;                      % Distance between joint 1 and joint 2
a2 = 146.05;                    % Distance between joint 2 and joint 3
a3 = 187.325;                   % Distance between joint 3 and joint 4
d4 = 34;                        % Distance between joint 4 and joint 5
d5 = 68;                        % Distance between joint 4 and end effector

% Joint limits
lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; % Upper joint limits in radians (grip in mm)

% Target orientation
r11 = T0e(1,1); r12 = T0e(1,2); r13 = T0e(1,3); 
r21 = T0e(2,1); r22 = T0e(2,2); r23 = T0e(2,3);
r31 = T0e(3,1); r32 = T0e(3,2); r33 = T0e(3,3);

%Target position
x = T0e(1,4);
y = T0e(2,4);
z = T0e(3,4);

% Wrist center position
xc = x-d5*r13;
yc = y-d5*r23;
zc = z-d5*r33;

distc = sqrt(xc^2+yc^2+(zc-d1)^2);
z_diff = atan2(yc,xc)-atan2(r23,r13);
if (distc > a2+a3) || (distc < a3-a2) || (abs(z_diff) > 0.1)
    isPos = false;
    q = [0,0,0,0,0];
    return
end

% Shoulder/elbow inverse orientation
if xc == 0 && yc == 0
    q1 = 0;
else
    q1 = atan2(yc,xc);
end

l = (a2^2+a3^2-xc^2-yc^2-(zc-d1)^2)/(2*a2*a3);
q3 = atan2(l,sqrt(1-l^2));
q2 = pi/2-atan2(zc-d1,sqrt(xc^2+yc^2))-atan2(a3*cos(q3),a2-a3*sin(q3));

% Wrist inverse orientation
c1=cos(q1);s1=sin(q1);
c2=cos(q2);s2=sin(q2);
c3=cos(q3);s3=sin(q3);

c23=c2*c3-s2*s3;
s23=s2*c3+s3*c2;

s4=-c1*s23*r13-s1*s23*r23-c23*r33;
c4=c1*c23*r13+s1*c23*r23-s23*r33;

s5=s1*r11-c1*r21;
c5=s1*r12-c1*r22;

q4=atan2(s4,c4);
q5=atan2(s5,c5);

q = [q1 q2 q3 q4 q5];
isPos = true;
end