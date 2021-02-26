d1 = 76.2;
a2 = 146.05;
a3 = 187.325;
d4 = 34;
d5 = 68;

syms q1 q2 q3 q4 q5
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

T03 = A*B*C;
T3e = D*E;
