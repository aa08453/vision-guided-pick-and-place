function T = exp_mat_homogenous(screw_axis, theta)
    w = screw_axis(1:3);
    v = screw_axis(4:6);

    if isa(theta, 'sym') || isa(screw_axis, 'sym')
        T = sym(eye(4));
    else
        T = eye(4);
    end

    w_skew = [  0,    -w(3),  w(2);
               w(3),   0,    -w(1);
              -w(2),  w(1),   0   ];

    R = eye(3) + sin(theta)*w_skew + (1-cos(theta))*w_skew^2;
    G = eye(3)*theta + (1-cos(theta))*w_skew + (theta-sin(theta))*w_skew^2;

    T(1:3, 1:3) = R;
    T(1:3, 4)   = G * v;
end




% function [T] = exp_mat_homogenous(screw_axis,theta)
% %UNTITLED8 Summary of this function goes here
% %   Detailed explanation goes here
% 
%     if isa(theta, 'sym') || isa(screw_axis, 'sym')
%         T = sym(eye(4));
%         G_eye = sym(eye(3));
%     else
%         T = eye(4);
%         G_eye = eye(3);
%     end
% 
% 
%     exp = rodro(screw_axis(1:3), theta);
% 
%     v = screw_axis(4:6);
%     T(1:3, 1:3) = exp;
% 
%     w_skew = skew_mat(screw_axis(1), screw_axis(2), screw_axis(3));
% 
% 
%     G = G_eye*theta + (1-cos(theta))*w_skew + (theta - sin(theta))*w_skew^2;
%     T(1:3, 4) = G * v;
% 
% end