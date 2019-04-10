clear
%initial variables
qinit1 = -pi/4;
qinit2 = pi/4;
qdotinit1 = 0;
qdotinit2 = qdotinit1;
q1f = pi/4;
q2f = 10*pi/180;

t = 10;
[t, a] = ode45(@hw5dynp2a, [0 t], [qinit1 qinit2 0 0]);

qdot1 = a(:,3);
qdot2 = a(:,4);
q1 = a(:,1);
q2 = a(:,2);

subplot(2,2,1)
plot(t, q1)
xlabel('time (sec)')
ylabel('q1 (rad)')
%axis equal

subplot(2,2,2)
plot(t, q2)
xlabel('time (sec)')
ylabel('q2 (rad)')
%axis equal

subplot(2,2,3)
plot(t, qdot1)
xlabel('time (sec)')
ylabel('qdot1 (rad/sec)')
%axis equal

subplot(2,2,4)
plot(t, qdot2)
xlabel('time (sec)')
ylabel('qdot2 (rad/sec)')
%axis equal