function [J] = calcJacobian(q,joint)

d1 = 76.2;
a2 = 146.05;
a3 = 187.325;
d4 = 34;
d5 = 68;
lg = 0;

A = [cos(q(1)) 0 -sin(q(1)) 0; 
     sin(q(1)) 0 cos(q(1)) 0;
     0 -1 0 d1; 
     0 0 0 1];

B = [sin(q(2)) cos(q(2)) 0 a2*sin(q(2));
    -cos(q(2)) sin(q(2)) 0 -a2*cos(q(2));
     0 0 1 0; 
     0 0 0 1];

C = [-sin(q(3)) -cos(q(3)) 0 -a3*sin(q(3));
     cos(q(3)) -sin(q(3)) 0 a3*cos(q(3));
     0 0 1 0;
     0 0 0 1];

D = [sin(q(4)) 0 cos(q(4)) d4*cos(q(4));
     -cos(q(4)) 0 sin(q(4)) d4*sin(q(4));
     0 -1 0 0;
     0 0 0 1];

E = [cos(q(5)) -sin(q(5)) 0 0;
     sin(q(5)) cos(q(5)) 0 0;
     0 0 1 (d5 - d4);
     0 0 0 1];

F = [1 0 0 0;
     0 1 0 0;
     0 0 1 lg;
     0 0 0 1];

T01 = A;
T02 = A*B;
T03 = A*B*C;
T04 = A*B*C*D;
T05 = A*B*C*D*E;
T0e = A*B*C*D*E*F;

x00 = [0 0 0]';
x01 = T01(1:3,4);
x02 = T02(1:3,4);
x03 = T03(1:3,4);
x04 = T04(1:3,4);
x05 = T05(1:3,4);
x0e = T0e(1:3,4);

z00 = [0 0 1]';
z01 = T01(1:3,3);
z02 = T02(1:3,3);
z03 = T03(1:3,3);
z04 = T04(1:3,3);
z05 = T05(1:3,3);

X = [x00,x01,x02,x03,x04,x05,x0e];
x = X(:,joint);

J1v = cross(z00,(x - x00));
J2v = cross(z01,(x - x01));
J3v = cross(z02,(x - x02));
J4v = cross(z03,(x - x03));
J5v = cross(z04,(x - x04));
Jev = cross(z05,(x - x05));

J1a = z00;
J2a = z01;
J3a = z02;
J4a = z03;
J5a = z04;
Jea = z05;

Jv = horzcat(J1v,J2v,J3v,J4v,J5v,Jev);
Ja = horzcat(J1a,J2a,J3a,J4a,J5a,Jea);
J = vertcat(Jv,Ja);
end
