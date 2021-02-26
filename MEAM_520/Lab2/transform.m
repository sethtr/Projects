function T = transform(T0e)
x = T0e(1,4);
y = T0e(2,4);
z = T0e(3,4);
R = T0e(1:3,1:3);
q1 = atan2(y,x);

theta = atan2(R(2,3),R(1,3)) - q1;


A = [cos(theta) sin(theta) 0; 
    -sin(theta) cos(theta) 0;
    0 0 1];
R = A * R;


T = [R(1,1) R(1,2) R(1,3) x;
    R(2,1) R(2,2) R(2,3) y;
    R(3,1) R(3,2) R(3,3) z;
    0 0 0 1];