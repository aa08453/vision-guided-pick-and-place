function [T] = exp_mat_homogenous(screw_axis,theta)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

    if isa(theta, 'sym') || isa(S, 'sym')
        T = sym(eye(4));
    else
        T = eye(4);
    end


    exp = rodro(screw_axis(1:3), theta);
    v = screw_axis(4:6);
    T(1:3, 1:3) = exp;
    T(1:3, 4) = exp * v;

end