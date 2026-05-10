function [x,y,z,R] = pincherFK(jointAngles) 
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

d = [13.7 0 0 0];
a = [0 10.5 10.5 11];
alpha = [pi/2, 0, 0, 0];


T = DH(jointAngles, d, a, alpha);
x = T(1,4);
y = T(2,4);
z = T(3,4);
R = T(1:3, 1:3);

end