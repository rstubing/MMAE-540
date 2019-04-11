clear
%initial variables
qinit1 = -pi/4;
qinit2 = pi/4;
qdotinit1 = 0;
qdotinit2 = qdotinit1;
%q1f = 2*sin(2*t);
%q2f = sin(t);

%running ode45, t = time in seconds
t = 10;
[t, a] = ode45(@hw6dynp1a, [0 t], [qinit1 qinit2 0 0]);

%giving value to joint variables
qdot1 = a(:,3);
qdot2 = a(:,4);
q1 = a(:,1);
q2 = a(:,2);

%extra stuff for tracking torque (put it here becuase I couldn't get it to
%work right in the function
kp = 10;
kd = 50;

tau1 = -kp*q1 - kd*qdot1;
tau2 = -kp*q2 - kd*qdot2;
max(tau1)
min(tau1)
max(tau2)
min(tau2)

%plots
figure(1)
subplot(2,2,1)
plot(t, q1 , '-b', t, 2*sin(2*t), '--r')
xlabel('time (sec)')
ylabel('q1 (rad)')
%axis equal

subplot(2,2,2)
plot(t, q2, '-b', t, sin(t), '--r')
xlabel('time (sec)')
ylabel('q2 (rad)')
%axis equal

subplot(2,2,3)
plot(t, qdot1, '-b', t, 4*cos(2*t), '--r')
xlabel('time (sec)')
ylabel('qdot1 (rad/sec)')
%axis equal

subplot(2,2,4)
plot(t, qdot2, '-b', t, cos(t), '--r')
xlabel('time (sec)')
ylabel('qdot2 (rad/sec)')
%axis equal
