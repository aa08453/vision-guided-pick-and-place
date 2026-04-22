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

% This is our "pick position"

% Outisde of a radius of 20, we cannot perform picks

points(1).x = 15; points(1).y = 0; points(1).z = 2; 
points(2).x = 20; points(2).y=5; points(2).z = 10;
points(3).x = 15; points(3).y=10; points(3).z = 6; 
points(4).x = 15; points(4).y=0; points(4).z = 20; 


angles = [-pi/2, NaN, -pi/2, NaN];
grip = [1.2,1.2,0, 0];

% moveIKSim(points, 2);

arb = Arbotix('port', 'COM4', 'nservos', 5);
arb.setpos(5, 0, 50);


zeroConfig(arb, 50)
pause(2);
moveIKReal(arb, points, angles, grip)
