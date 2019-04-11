%differential drive robot, this works!

clear all

t = 15;
[t, s] = ode45(@mobile5dyn, [0 t], [0 0 0]);

x = s(:,1);
y = s(:,2);
theta = s(:,3);

figure(1)
quiver(x, y, cos(theta), sin(theta), 'color', [0 0 1])
xlabel('x-axis')
ylabel('y-axis')
title('path of differential drive robot with velocity vectors')

figure(2)
plot(x,y)
xlabel('x-axis')
ylabel('y-axis')
title('path of differential drive robot')