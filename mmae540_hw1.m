clear
home

%initial conditions
L1 = 4;
L2 = 3;
time_step = 0.001;
x(1) = 0;
y(1) = 3;

%arm configuration 1 (page 3 of notes, q calculated from x)
q2(1) = -acos((x^2+y^2-L1^2-L2^2)/(2*L1*L2));
q1(1) = atan2(y(1),x(1)) - atan2(L2*sin(q2(1)), L1+L2*cos(q2(1)));
i=1;

%arm configuration 2
x(i) = L1*cos(q1(i)) + L2*cos(q1(i)+q2(i));
y(i) = L1*sin(q1(i)) + L2*sin(q1(i)+q2(i));
phi = atan2(y(i), x(i));
q1_alt(1) = 2*phi - q1(1);
q2_alt(1) = -q2(1);

%plots the two arms to check starting angles
figure(1); clf;
hold on;
line([0; L1*cos(q1(i))], [0; L1*sin(q1(i))]);
line([L1*cos(q1(i)), L1*cos(q1(i)) + L2*cos(q1(i) + q2(i))], [L1*sin(q1(i)), L1*sin(q1(i)) + L2*sin(q1(i) + q2(i))]);
line([0; L1*cos(q1_alt(i))], [0; L1*sin(q1_alt(i))]);
line([L1*cos(q1_alt(i)), L1*cos(q1_alt(i)) + L2*cos(q1_alt(i) + q2_alt(i))], [L1*sin(q1_alt(i)), L1*sin(q1_alt(i)) + L2*sin(q1_alt(i) + q2_alt(i))]);
axis equal

%for elbow left comment this out
q1(1) = q1_alt;
q2(1) = q2_alt;

time(i) = 0;
while x(i) <= 3
    i = i + 1;
    %compute the Jacobian as a function of the joint angles, q
    J = [ -L1*sin(q1(i-1)) - L2*sin(q1(i-1) + q2(i-1)), -L2*sin(q1(i-1) +q2(i-1));...
        L1*cos(q1(i-1)) + L2*cos(q1(i-1) + q2(i-1)), L2*cos(q1(i-1) + q2(i-1))];
    
    %joint velocities are related to the end point velocities as the
    %inverse of the jacobian
    qdot = inv(J)*[1;0];
    q1dot(i) = qdot(1);
    q2dot(i) = qdot(2);
    
    %this increments the joint angles and then computes the x(i) and y(i)
    %could also try to increment the end effector and compute the joint
    %angles
    q1(i) = q1(i-1) + q1dot(i)*time_step;
    q2(i) = q2(i-1) + q2dot(i)*time_step;
    x(i) = L1*cos(q1(i)) + L2*cos(q1(i) + q2(i));
    y(i) = L1*sin(q1(i)) + L2*sin(q1(i) + q2(i));
    time(i) = time(i-1) + time_step;
end

%now change velocities
while x(i) <= 5
    i = i + 1;
    J = [ -L1*sin(q1(i-1)) - L2*sin(q1(i-1) + q2(i-1)), -L2*sin(q1(i-1) +q2(i-1));...
        L1*cos(q1(i-1)) + L2*cos(q1(i-1) + q2(i-1)), L2*cos(q1(i-1) + q2(i-1))];
    qdot = inv(J)*[sqrt(.5); -sqrt(.5)];
    q1dot(i) = qdot(1);
    q2dot(i) = qdot(2);
    q1(i) = q1(i-1) + q1dot(i)*time_step;
    q2(i) = q2(i-1) + q2dot(i)*time_step;
    x(i) = L1*cos(q1(i)) + L2*cos(q1(i) + q2(i));
    y(i) = L1*sin(q1(i)) + L2*sin(q1(i) + q2(i));
    time(i) = time(i-1) + time_step;
end
while y(i) >= 0
    i = i + 1;
    J = [ -L1*sin(q1(i-1)) - L2*sin(q1(i-1) + q2(i-1)), -L2*sin(q1(i-1) +q2(i-1));...
        L1*cos(q1(i-1)) + L2*cos(q1(i-1) + q2(i-1)), L2*cos(q1(i-1) + q2(i-1))];
    qdot = inv(J)*[0; -1];
    q1dot(i) = qdot(1);
    q2dot(i) = qdot(2);
    q1(i) = q1(i-1) + q1dot(i)*time_step;
    q2(i) = q2(i-1) + q2dot(i)*time_step;
    x(i) = L1*cos(q1(i)) + L2*cos(q1(i) + q2(i));
    y(i) = L1*sin(q1(i)) + L2*sin(q1(i) + q2(i));
    time(i) = time(i-1) + time_step;
end

lw = 2; %linewidth for better plotting
figure(1)
subplot(2,1,1)
plot(time, q1dot, 'LineWidth', lw)
axis tight
ylabel('Joint 1 velocity (rad/s)');
grid on
title('Joint velocities for homework 1.1f')
subplot(2,1,2)
plot(time, q2dot, 'LineWidth', lw)
grid on
axis tight
xlabel('Time (s)')
ylabel('Joint 2 Velocity (rad/s)')

figure(2)
plot(x, y, 'linewidth', lw)
hold on
plot(x, y, 'r', 'linewidth', lw)
xlabel('X position of the end effector (no units)')
ylabel('Y position of the end effector (no units)')
title('End effector position for homework 1.1f')
axis equal
grid on
