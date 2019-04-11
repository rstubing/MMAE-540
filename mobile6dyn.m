function dsdt = mobile6dyn(t,s)
%dynamics for tractor trailer mobile robot that attempts to use
%a differential drive tractor with several trailers

%set the variables to be integrated
x = s(1);
y = s(2);
theta = s(3);
psi1 = s(4);
psi2 = s(5);

%controllable variables
u = 2;      %linear speed of tractor

%need to make alpha a function of the lead tractor
if t<= 2
    alpha = 0; %angle between body heading and u
else
    alpha = pi/8;
end

%lengths of each tractor connector
L0 = 1;
L1 = L0;
L2 = L0;

%kinematic equations
L = .5;
r = .1;
phidot1 = 2;
phidot2 = 2;

Rinv = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
J1 = [.5 .5; 0 0; 1/(2*L) -1/(2*L)];
J2 = [r 0; 0 r];
phidot = [phidot1; phidot2];

etadot = Rinv*J1*J2*phidot;

xdot = etadot(1);
ydot = etadot(2);
thetadot = etadot(3);

u = phidot1*r;

psidot1 = (u/L0)*sin(alpha) - (u/L1)*cos(alpha)*sin(psi1);
psidot2 = (u/L1)*cos(alpha)*sin(psi1) - (u/L2)*cos(alpha)*cos(psi1)*sin(psi2);

dsdt = [xdot; ydot; thetadot; psidot1; psidot2];