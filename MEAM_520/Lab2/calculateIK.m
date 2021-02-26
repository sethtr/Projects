function [q isPos] = calculateIK(T0e)
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

% Target Z axis orientation
r13 = T0e(1,3); 
r23 = T0e(2,3);
r33 = T0e(3,3);

% Target position
x = T0e(1,4);
y = T0e(2,4);
z = T0e(3,4);

% Wrist center position
xc = x-d5*r13;
yc = y-d5*r23;
zc = z-d5*r33;

% Check for valid postions & orientations
distc = sqrt(xc^2+yc^2+(zc-d1)^2);
z_alignment = atan2(yc,xc)-atan2(r23,r13);
transformation = false;

if (distc > a2+a3) || (distc < a3-a2) % Wrist position
    isPos = false;
    q = [];
    return

elseif (abs(z_alignment) > 0.1) % Wrist orientation
    T0e = transform(T0e);
    transformation = true;
    
    % New target Z axis orientation
    r13 = T0e(1,3); 
    r23 = T0e(2,3);
    r33 = T0e(3,3);

    % New target position
    x = T0e(1,4);
    y = T0e(2,4);
    z = T0e(3,4);

    % New wrist center position
    xc = x-d5*r13;
    yc = y-d5*r23;
    zc = z-d5*r33;
end

% Shoulder & elbow inverse orientation
singularity = false;
if xc == 0 && yc == 0
    q1 = 0;
    singularity = true; 
else
    q1 = atan2(yc,xc);
end

l = (a2^2+a3^2-xc^2-yc^2-(zc-d1)^2)/(2*a2*a3);
q3 = atan2(l,sqrt(1-l^2));
q2 = pi/2-atan2(zc-d1,sqrt(xc^2+yc^2))-atan2(a3*cos(q3),a2-a3*sin(q3));

% Wrist inverse orientation
A = [cos(q1) 0 -sin(q1) 0; 
    sin(q1) 0 cos(q1) 0;
    0 -1 0 d1; 
    0 0 0 1];

B = [sin(q2) cos(q2) 0 a2*sin(q2); 
    -cos(q2) sin(q2) 0 -a2*cos(q2);
    0 0 1 0; 
    0 0 0 1];

C = [-sin(q3) -cos(q3) 0 -a3*sin(q3);
    cos(q3) -sin(q3) 0 a3*cos(q3); 
    0 0 1 0; 
    0 0 0 1];

T3e = (A*B*C)'*T0e;
q4 = atan2(T3e(2,3),T3e(1,3));
q5 = atan2(-T3e(3,1),-T3e(3,2));

q = [q1 q2 q3 q4 q5];

% Joint limit check
rows = [];
for r = 1:size(q,1)
    for ii = 1:5
        if q(r,ii) < lowerLim(ii) || q(r,ii) > upperLim(ii)
            rows = [rows, r];
        end
    end
end
rows = unique(rows);
q(rows,:) = [];

% Determine isPos value
if transformation == true
    isPos = false;
else
    isPos = ~isempty(q);
end

% Adujust q if there is a singularity
if singularity == true && ~isempty(q)
    q(1) = NaN;
end

end