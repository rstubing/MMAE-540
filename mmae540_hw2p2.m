%Richard Stubing
%MMAE 540 HW #2

%Problem 2

%D. Cylindrical RPP: Workspace is 2 concentric cylinders
L1 = 1;
L2 = 1;
theta = 360;

xmin = -L1/2;
xmax = L1/2;
ymin = -L2/2;
ymax = L2/2;

x = [xmin, xmax];
y = [ymin, ymax];
z = [0, 2*pi];

[X, Y, Z] = cylinder;

surf(X,Y,Z)


