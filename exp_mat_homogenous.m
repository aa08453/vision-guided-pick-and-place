function [T] = exp_mat_homogenous(screw_axis,theta)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
    exp = RODRO(screw_axis(1:3), theta);
    v = screw_axis(4:6);
    T = zeros(4,4);
    T(1:3, 1:3) = exp;
    T(1:4, 4) = exp * v;

end