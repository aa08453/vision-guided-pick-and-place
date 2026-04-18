clc; clear; close all;

% points(1).x = 0;
% points(1).y = 0;
% points(1).z = 45.7;
% 
% points(2).x = 15;
% points(2).y = 10;
% points(2).z = 15;
% 
% 
% points(3).x = 5;
% points(3).y = 10;
% points(3).z = 25;
% 
% 
% points(4).x = 15;
% points(4).y = 0;
% points(4).z = 30;


points(1).x = -20;
points(1).y = 0;
points(1).z = 5;

moveIKSim(points, 1);


% arb = Arbotix('port', 'COM4', 'nservos', 5);
% moveIKReal(arb, points, 1)