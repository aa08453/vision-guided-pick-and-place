clc; close all; clear;
% Verify that your robot has been built properly by using the showdetails or
% show function. The showdetails function lists all the bodies of the robot 
% in the MATLAB® command window. The show function displays the robot with 
% a specified configuration (home by default).
numTests = 100000;

numRight = 0;
figure;
set(groot,'DefaultFigureWindowStyle','docked')

points = zeros(numTests, 3);
lb = 5/6 .*[-pi,-pi,-pi,-pi];
ub = 5/6 .*[pi,pi,pi,pi];
% Replace rand with lhsdesign for better coverage
samples = lhsdesign(numTests, 4);  % Latin Hypercube
configs = lb + (ub - lb) .* samples;


% Remove scatter3 inside loop, just collect points
for i = 1:numTests
    curConfig = configs(i,:);
    [x, y, z, ~] = pincherFK(curConfig, 0);
    points(i,:) = [x, y, z];
end
validPoints = points(points(:,3) >= 0, :);
[k, av] = convhull(validPoints, 'Simplify', true);
% [k, av] = convhull(points, 'Simplify', true);
fprintf("Workspace convex hull volume: %.2f cm³\n", av);

trisurf(k, points(:,1), points(:,2), points(:,3), ...
    'FaceColor', [0.2, 0.5, 0.8], ...
    'FaceAlpha', 0.3, ...
    'EdgeColor', 'none');

hold on;
scatter3(points(:,1), points(:,2), points(:,3), 2, 'k', 'filled');
xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
title(sprintf('Phantom X Pincher Workspace (%d samples)', numTests));
axis equal; grid on; view(0, 90);

% scatter3(points(:,1), points(:,2), points(:,3))
% 
% [k, av] = convhull(points, 'Simplify', true);
% trisurf(k, points(:,1), points(:,2), points(:,3), 'Facecolor', [0.2, 0.3, 0.2])
% 
