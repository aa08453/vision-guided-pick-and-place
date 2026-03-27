function dhJointAngles = servo2dh(jointAngles, deg)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
dhJointAngles = jointAngles;
if (deg == true)
dhJointAngles = deg2rad(dhJointAngles);
end
dhJointAngles(2) = jointAngles(2) + deg2rad(90);
end