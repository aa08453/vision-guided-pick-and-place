
function [jointAngles] = getCurrentPose(arb)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
q = arb.getpos();
jointAngles = servo2dh(q(1:4),false);
end