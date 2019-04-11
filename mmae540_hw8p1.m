%plot the workspace of 2-link revolute joint manipulator
clear
%coordinates for obstacle
x1 = [.25 .75 .75 .25 .25];
y1 = [.45 .45 -.05 -.05 .45];

figure(1)
plot(x1,y1)
grid on
axis equal
title('obstacle inside workspace')

%workspace is circle of radius 2
viscircles([0 0], 2);

%configuration space with obstacle
figure(2)
L1 = 1;
L2 = 1;
k=1;
for i=-180:2:180
    q1 = i*(pi/180);
    for j = -180:2:180
        q2 = j*(pi/180);
        X = L1*cos(q1) + L2*cos(q1+q2);
        Y = L1*sin(q1) + L2*sin(q1+q2);
        X1 = L1*cos(q1);
        X2 = X1 + L2*cos(q1+q2);
        Y1 = L1*sin(q1);
        Y2 = Y1 + L2*sin(q1+q2);
        for m = 0.01:0.05:1
            if (((0.25<=X1*m)&&(X1*m<=0.75)) || ((0.25<=X2*m)&&(X2*m<=0.75))) && (((-0.05<=Y1*m)&&(Y1*m<=0.45)) || ((-0.05<=Y2*m)&&(Y2*m<=0.45)))
            %if (((0.25<=X1)&&(X1<=0.75)) || ((-0.05<=Y1)&&(Y1<=0.45))) || (((0.25<=X2)&&(X2<=0.75)) && ((-0.05<=Y2)&&(Y2<=0.45)))
            %if (0.25<=X1)&&(X1<=0.75) || ((((0.25<=X1)&&(X1<=0.75)) || ((-0.05<=Y1)&&(Y1<=0.45))) || (((0.25<=X2)&&(X2<=0.75)) && ((-0.05<=Y2)&&(Y2<=0.45)))) && (-0.05<=Y1)&&(Y1<=0.45) || (-0.05<=Y2)&&(Y2<=0.45)
                x(k) = q1*(180/pi);
                y(k) = q2*(180/pi);
                k = k+1;
            end
        end
     
    end
    
end

scatter(x,y)
grid on
axis equal
xlim([-180 180])
ylim([-180 180])
xlabel('Joint Angle 1')
ylabel('Joint Angle 2')
title('Configuration Space with Obstacle')

        