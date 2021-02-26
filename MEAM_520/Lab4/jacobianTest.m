syms q1_dot q2_dot q3_dot q4_dot q5_dot q6_dot

q = [0 0 pi/2 0 0 0];
joint = 6;

J = calcJacobian(q,joint);

test1 = simplify(J*[q1_dot 0 0 0 0 0]');
test2 = simplify(J*[0 q2_dot 0 0 0 0]');
test3 = simplify(J*[0 0 q3_dot 0 0 0]');
test4 = simplify(J*[0 0 0 q4_dot 0 0]');
test5 = simplify(J*[0 0 0 0 q5_dot 0]');
test6 = simplify(J*[0 0 0 0 0 q6_dot]');