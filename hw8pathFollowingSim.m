clear
clc

poseR = [1; 0; 0];
poseI = [0; 1; degtorad(10)];
tspan = [0, 5];
[T, Y] = ode45(@PathFollowingSim, tspan, poseI,[], poseR);

%%
figure(1); clf;
quiver(Y(:,1), Y(:,2), cos(Y(:,3)), sin(Y(:,3)))
axis equal
grid on
xlabel('x (m)')
ylabel('y (m)')