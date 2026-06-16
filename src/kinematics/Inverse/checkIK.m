clc; clear; close all;

% This is our "pick position"

% Outisde of a radius of 20, we cannot perform picks
points(1).x = 0; points(1).y=32; points(1).z = 13.7; 
points(2).x = 0; points(2).y =25; points(2).z = 20;
points(3).x = -32; points(3).y = 0; points(3).z = 13.7; % pick
points(4).x = -25; points(4).y = 0; points(4).z = 20;
points(5).x = 0; points(5).y=-30; points(5).z = 13.7;
points(6).x = 0; points(6).y = -25; points(6).z = 20;
% points(4).x = 0; points(4).y=32; points(4).z = 13.7; % place
points(1).x = 32; points(1).y=0; points(1).z = 13.7; 
% 
% 
angles = [NaN, NaN, NaN, NaN, NaN, NaN, NaN];
grip = [2.2,2.2,2.2,2.2,2.2, 2.2, 2.2];
 
% 
% 
% points(1).x = 15; points(1).y=0; points(1).z = 20; 
% points(2).x = 32; points(2).y = 0; points(2).z = 13.7; 


% angles = [NaN,0];
% grip = [0,0];




% moveIKSim(points, 5);

arb = Arbotix('port', 'COM5', 'nservos',5);
arb.setpos(5, 0, 50);
pause(2);


moveIKReal(arb, points, angles, grip)
