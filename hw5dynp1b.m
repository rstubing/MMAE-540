function dxdt = hw5dynp1b(t,x)

m1 = 5;
m2 = m1;
l1 = 1;
l2 = l1;

lc1 = l1/2;
lc2 = l2/2;
I1 = (m1*l1^2)/12;     
I2 = (m2*l2^2)/12;      
g = -9.8;

q1 = x(1);
q2 = x(2);
qdot1 = x(3);
qdot2 = x(4);

H11 = m1*lc1^2 + I1 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2)) + I2;
H22 = m2*lc2^2 + I2;
H12 = m2*l1*lc2*cos(q2) + m2*lc2^2 + I2;
h = m2*l1*lc2*sin(q2);
G1 = m1*lc1*g*cos(q1) + m2*lc2*g*cos(q1 + q2) + m2*l1*g*cos(q1);
G2 = m2*lc2*g*cos(q1 + q2);

H = [H11 H12; H12 H22];
C = [-h*qdot2 -h*qdot1-h*qdot2; h*qdot1 0];
G = [G1; G2];
T = [-2*exp(-3*t); -10*exp(-t)];

qdd = H\(T - C*[qdot1; qdot2] - G);

dxdt = [qdot1 qdot2 qdd(1) qdd(2)]';
end