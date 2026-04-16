function [jointAngles] = getCurrentPose(arb)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
q = arb.getpos();
jointAngles = real2sim(q(1:4));
end