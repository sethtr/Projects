function [jointPositions,T0e] = calculateFK(q)
% CALCULATEFK - 
%
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   q - 1x6 vector of joint inputs [q1,q2,q3,q4,q5,lg]
%
% OUTPUT:
%   jointPositions - 6 x 3 matrix, where each row represents one 
%                    joint along the robot. Each row contains the [x,y,z]
%                    coordinates of the respective joint's center (mm). For
%                    consistency, the first joint should be located at 
%                    [0,0,0]. These values are used to plot the robot.
%   T0e            - a 4 x 4 homogeneous transformation matrix, 
%                    representing the end effector frame expressed in the 
%                    base (0) frame
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Lynx Dimensions in mm
L1 = 76.2;    % distance between joint 0 and joint 1
L2 = 146.05;  % distance between joint 1 and joint 2
L3 = 187.325; % distance between joint 2 and joint 3
L4 = 34;      % distance between joint 3 and joint 4
L5 = 34;      % distance between joint 4 and center of gripper

%% Your code here
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
lg = q(6);


A = [cos(q1) 0 -sin(q1) 0; 
    sin(q1) 0 cos(q1) 0;
    0 -1 0 L1; 
    0 0 0 1];

B = [cos(q2) sin(q2) 0 -L2*sin(q2); 
    -sin(q2) cos(q2) 0 -L2*cos(q2);
    0 0 1 0; 
    0 0 0 1];

C = [cos(q3) sin(q3) 0 L3*cos(q3);
    -sin(q3) cos(q3) 0 -L3*sin(q3); 
    0 0 1 0; 
    0 0 0 1];

D = [-sin(q4) 0 cos(q4) L4*cos(q4); 
    -cos(q4) 0 -sin(q4) -L4*sin(q4); 
    0 -1 0 0; 
    0 0 0 1];

E = [cos(q5) -sin(q5) 0 0; 
    sin(q5) cos(q5) 0 0; 
    0 0 1 (L5 + lg); 
    0 0 0 1];

T01 = A;
T02 = A*B;
T03 = A*B*C;
T04 = A*B*C*D;
T0e = A*B*C*D*E;

jointPositions = [0 0 0; T01(1:3, 4)'; T02(1:3, 4)'; 
    T03(1:3, 4)'; T04(1:3, 4)'; T0e(1:3, 4)'];
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end