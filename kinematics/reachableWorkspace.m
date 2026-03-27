clc; close all; clear;
% Verify that your robot has been built properly by using the showdetails or
% show function. The showdetails function lists all the bodies of the robot 
% in the MATLAB® command window. The show function displays the robot with 
% a specified configuration (home by default).
numTests = 1000;

numRight = 0;
figure;
set(groot,'DefaultFigureWindowStyle','docked')

points = zeros(numTests, 3);
lb = 5/6 .*[-pi,-pi,-pi,-pi];
ub = 5/6 .*[pi,pi,pi,pi];
configs = lb + (ub - lb) .* rand(numTests, 4);



for i = 1:numTests    
    hold on;
    grid on;
    box on;
    curConfig = configs(i,:);
    [x,y,z,R] = pincherFK(curConfig, 0);
    points(i,:) = [x, y, z];
    scatter3(x,y,z);
end
scatter3(points(:,1), points(:,2), points(:,3))

[k, av] = convhull(points, 'Simplify', true);
trisurf(k, points(:,1), points(:,2), points(:,3), 'Facecolor', [0.2, 0.3, 0.2])

