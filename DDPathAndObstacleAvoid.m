%obstacle avoiding and path planning with differential drive robot

clear; clc; close all;

%define the goal and potential field constants
goal = [25; 30]; %goal location
d = 3;          %distance that the attractive force levels off
rho_o = 2;      %distance of influence of an obstacle
eta = 100;      %Strength of repulsive field
beta = 0.1;    %step size to change steering angle
L = 5;          %vehicle length
r = 2;          %wheel radius


%define the obstacle locations, vertex vectors
xx1 = [5; 5];   %center of obstacle 1
xx2 = [15; 12]; %center of obstacle 2
xx3 = [20; 20]; %center of obstacle 3
xob1 = [ -2.5 3 2 -2.5 -2.5; 3 2 7 6 3];
xob2 = [ -4 3 3 -4; 0 -2 2 0];
xob3 = [ -3 3 4 3 -3 -6 -3; -2 -2 0 4 4 0 -2];
xobst1 = [xob1(1,:) + xx1(1,1); xob1(2,:) + xx1(2,1)];
xobst2 = [xob2(1,:) + xx2(1,1); xob2(2,:) + xx2(2,1)];
xobst3 = [xob3(1,:) + xx3(1,1); xob3(2,:) + xx3(2,1)];

%initialize the parameters of the vehicle
index = 1;
x(1) = 0;
y(1) = 0;
theta(1) = 0;

%compute the algorithm while the distance from the end effector to the
%goal is greater than 3 m
%while index<1000
    while sqrt((x(index) - goal(1))^2 + (y(index) - goal(2))^2) > 3
    
        %goal attractive forces
        config = [x(index) - goal(1); y(index) - goal(2)];
        Fatt = mmae540_fatt(d, 3, config);

        %obstacle 1 repulsive forces
        Frep1 = mmae540_ffrep(rho_o, eta, [x(index); y(index)], xobst1);

        %obstacle 2 repulive forces
        Frep2 = mmae540_ffrep(rho_o, eta, [x(index); y(index)], xobst2);

        %obstacle 3 repulsive forces
        Frep3 = mmae540_ffrep(rho_o, eta, [x(index); y(index)], xobst3);

        %sum of the forces
        Fsum = Fatt + Frep1 + Frep2 + Frep3;

        desireddirection = Fsum/norm(Fsum);

        actualdirection = [cos(theta(index)); sin(theta(index))];

        errordirection = acos(dot(desireddirection, actualdirection)/...
            (norm(desireddirection)*norm(actualdirection)));

        %compute the sign of the angle
        c = cross([desireddirection; 1], [actualdirection; 1]); 
        steering_sign = -sign(c(3));

        %set the steering angle to be a function of that angle
        delta = beta*steering_sign*real(errordirection);

        %vehicle kinematics
        Rinv = [cos(theta(index)) -sin(theta(index)) 0; sin(theta(index)) cos(theta(index)) 0; 0 0 1];
        J1 = [.5 .5; 0 0; 1/(2*L) -1/(2*L)];
        J2 = [r 0; 0 r];

        if delta < pi/32 
            psidot1 = 1; psidot2 = 1;
        else if delta > pi/32
                psidot1 = 80; psidot2 = 1;
            %else if delta > pi/24
                    %psidot1 = 8; psidot2 = 1;
                else if delta > (15*pi)/16
                        psidot1 = 1; psidot2 = 80;
                    end
                %end
            end
        end

        psidot = [psidot1; psidot2];
        etadot = Rinv*J1*J2*psidot;

        xdot = etadot(1);
        ydot = etadot(2);
        thetadot = etadot(3);

        timestep = 0.01;
        x(index+1) = x(index) + xdot*timestep;
        y(index+1) = y(index) + ydot*timestep;
        theta(index+1) = theta(index) + thetadot*timestep;

        index = index + 1;
    end
%end


%plot results
figure(1); hold on; grid on; axis equal
fill(xobst1(1,:), xobst1(2,:), 'r') %draw obstacle 1
fill(xobst2(1,:), xobst2(2,:), 'r') %draw obstacle 2
fill(xobst3(1,:), xobst3(2,:), 'r') %draw obstacle 3
plot(goal(1), goal(2), 'x-m', 'MarkerSize', 12, 'LineWidth', 2)
plot(x, y)
xlabel('X')
ylabel('Y')
title('Path planning applied to differential drive robot')