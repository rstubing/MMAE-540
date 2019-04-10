%initial variables
qinit1 = pi/6;
qinit2 = pi/3;
qdotinit1 = 0;
qdotinit2 = qdotinit1;

i = 1;
while i <= 4
    t = [1 2 5 10];
    [t, a] = ode45(@hw5dyn, [0 t(i)], [30*pi/180 60*pi/180 0 0]);

    qdot1 = a(:,3);
    qdot2 = a(:,4);
    q1 = a(:,1);
    q2 = a(:,2);

    x = L1*cos(q1) + L2*cos(q1 + q2);
    y = L1*sin(q1) + L2*sin(q1 + q2);

    figure(1)
    subplot(2,2,i)
    plot(x, y)
    xlabel('x')
    ylabel('y')
    title('End Effector Trajectory')
    
    i = i + 1;
end