function dsdt = mobile2(t,s)
%dynamics for tractor trailer mobile robot that follows a path into a
%circle

x = s(1);
y = s(2);
theta = s(3);
psi1 = s(4);
psi2 = s(5);

%controllable variables
u = 2;
if t<= 2
    alpha = 0; %angle between body heading and u
else
    alpha = pi/8;
end
L0 = 1;
L1 = L0;
L2 = L0;

%kinematic equations
xdot = u*cos(alpha)*cos(theta);
ydot = u*cos(alpha)*sin(theta);
thetadot = (u/L0)*sin(alpha);
psidot1 = (u/L0)*sin(alpha) - (u/L1)*cos(alpha)*sin(psi1);
psidot2 = (u/L1)*cos(alpha)*sin(psi1) - (u/L2)*cos(alpha)*cos(psi1)*sin(psi2);

dsdt = [xdot; ydot; thetadot; psidot1; psidot2];