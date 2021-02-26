syms d1 a2 a3 d4 d5 lg q1 q2 q3 q4 q5 q5 q6

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

D = [sin(q4) 0 cos(q4) d4*cos(q4);
     -cos(q4) 0 sin(q4) d4*sin(q4);
     0 -1 0 0;
     0 0 0 1];

E = [cos(q5) -sin(q5) 0 0;
     sin(q5) cos(q5) 0 0;
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

J1a = [0 0 1]';
J2a = T01(1:3,3);
J3a = T02(1:3,3);
J4a = T03(1:3,3);
J5a = T04(1:3,3);
Jea = T05(1:3,3);

x = simplify(T0e(1,4));
y = simplify(T0e(2,4));
z = simplify(T0e(3,4));

Jv = [diff(x, q1), diff(x, q2), diff(x, q3), diff(x, q4), diff(x, q5), diff(x, q6);  
      diff(y, q1), diff(y, q2), diff(y, q3), diff(y, q4), diff(y, q5), diff(y, q6); 
      diff(z, q1), diff(z, q2), diff(z, q3), diff(z, q4), diff(z, q5), diff(z, q6)];

Ja = horzcat(J1a,J2a,J3a,J4a,J5a,Jea);

J = vertcat(Jv,Ja);

