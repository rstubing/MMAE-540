%this runs the tractor trailer robot with a differential drive tractor
clear all

t = 10;
[t, s] = ode45(@mobile6dyn, [0 t], [0 0 0 0 0]);

x = s(:,1);
y = s(:,2);
theta = s(:,3);
psi1 = s(:,4);
psi2 = s(:,5);

L0 = 1;
L1 = L0;
L2 = L0;

%second tractor
x2 = x - L1*cos(theta + psi1);
y2 = y + L1*sin(theta + psi1);

%third tractor
x3 = x2 - L2*cos(theta + psi1 + psi2);
y3 = y2 + L2*sin(theta + psi1 + psi2);

%quiver plot of path and velocity vectors
figure(1)
quiver(x, y, cos(theta), sin(theta), 'color', [0 0 1])
hold on
quiver(x2, y2, cos(theta + psi1), sin(theta + psi1), 'color', [1 0 0])
hold on
quiver(x3, y3, cos(theta + psi1 + psi2), sin(theta + psi1 + psi2))
xlabel('x-axis')
ylabel('y-axis')
title('path of tractor-trailer robot with velocity vectors')
legend('lead tractor', 'middle car', 'end car')
axis equal
hold off

%plot of path of the robot
figure(2)
plot(x, y, x2, y2, x3, y3)
xlabel('x-axis')
ylabel('y-axis')
title('path of tractor trailer robot')
legend('lead tractor', 'middle car', 'end car')
axis equal