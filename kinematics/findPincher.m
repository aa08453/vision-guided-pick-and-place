function [x,y,z,R] = findPincher(arb)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
q = arb.getpos();
jointangles = servo2dh(q(1:4),false);
[x,y,z,R] = pincherFK(jointangles, false);
end