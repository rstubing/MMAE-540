function pose_dot = PathFollowingSim(~, pose, refPose)
x = pose(1);
y = pose(2);
theta = pose(3);
thetar = refPose(3);

theta_tilda = theta - thetar;
l = y; %because following x-axis
v = 1;
c = 0;
xi = 1/sqrt(2);
a = 1;
k2 = a^2;
k3 = 2*xi*a;
u = -k2*v*l - k3*abs(v)*theta_tilda;
w = u + v*cos(theta_tilda)*c/(1-c*l);
pose_dot = [v*cos(theta); v*sin(theta); w];