function dadt = hw7dynp1(t,a)

%variables
x = a(1);
y = a(2);
theta = a(3);

%final conditions
xr = 10;
yr = 2;
thetar = pi/4;
vr = 1;
wr = 10*pi/180;

%gain values
k1 = 10;
k2 = 0.5;
k3 = 0.5;

G = [cos(theta) 0; sin(theta) 0; 0 1];

e = [cos(theta) sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1]*[xr - x; yr - y; thetar - theta];

u = [vr*cos(e(3)) + k1*e(1); wr + k2*sin(vr)*e(2) + k3*e(3)];

%edot = [0 wr 0; -wr 0 vr; 0 0 0]*e + [1 0; 0 0; 0 1]*u;

zdot = G*u;

xdot = zdot(1);
ydot = zdot(2);
thetadot = zdot(3);

dadt = [xdot ydot thetadot]';
end

