%kinematics for differential drive robot, this works!

function dsdt = mobile5(t,s)

x = s(1);
y = s(2);
theta = s(3);       %theta is heading angle of the robot

L = 1;             %distance from each wheel to COM of robot
r = .5;            %radius of the wheels (change this to get robot to move faster without changing psi)
psidot1 = 2;        %rotational speed of wheel 1
psidot2 = 1;        %rotational speed of wheel 2


%kinematic equations
Rinv = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
J1 = [.5 .5; 0 0; 1/(2*L) -1/(2*L)];
J2 = [r 0; 0 r];
psidot = [psidot1; psidot2];

etadot = Rinv*J1*J2*psidot;

xdot = etadot(1);       %x-velocity
ydot = etadot(2);       %y-velocity
w = etadot(3);          %rate of change of heading angle, yaw-rate

dsdt = [xdot; ydot; w];