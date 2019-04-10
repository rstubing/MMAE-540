clear
home
L1 = 3; 
L2 = 4;
f = 1;
m = 1;
g = 9.8;
lc1 = 0.5;
lc2 = 0.5;

for i = 1:45
    q1(i) = (i-1)*8*pi/180;
    for j = 1:45
        q2(j) = (j-1)*8*pi/180;
        L = sqrt(L1^2 + L2^2 + 2*L1*L2*cos(q2(j)));
        if L == 0;
            beta = 0;
        else
            beta = asin((L2/L)*sin(pi-q2(j)));
        end
        
        F = [-f*cos(beta+q1(i)); -f*sin(beta + q1(i))];
        J1 = [-L1*sin(q1(i)) - L2*sin(q1(i) + q2(j)), -L2*sin(q1(i) + q2(j));...
                L1*cos(q1(i)) + L2*cos(q1(i) + q2(j)), L2*cos(q1(i) + q2(j))];
        J2 = [-lc1*sin(q1(i)) 0; -lc1*cos(q1(i)) 0];
        J3 = [-L1*sin(q1(i)) - lc2*sin(q1(i) + q2(i)), -lc2*sin(q1(i) +q2(i));...
                -L1*cos(q1(i)) + lc2*cos(q1(i) + q2(i)), lc2*cos(q1(i) + q2(i))];
        T = -J1'*F - J2'*[0; m*g];
        T1(i,j) = T(1);
        T2(i,j) = T(2);
    end
end

figure(1)
clf
subplot(2,1,1)
mesh(q2*180/pi, q1*180/pi, T1)
set(gca, 'ytick', [0 90 180 270 360])
set(gca, 'xtick', [0 90 180 270 360])
shading interp
colormap(gray)
axis tight
xlabel('Joint 2 Angle (rad)')
ylabel('Joint 1 Angle (rad)')
zlabel('Joint 1 Torque');
grid on
subplot(2,1,2)
mesh(q2*180/pi, q1*180/pi, T2)
set(gca, 'ytick', [0 90 180 270 360])
set(gca, 'xtick', [0 90 180 270 360])
shading interp

colormap(gray)
axis tight
xlabel('Joint 2 Angle(deg)')
ylabel('Joint 1 Angle(deg)')
zlabel('Joint 2 Torque');