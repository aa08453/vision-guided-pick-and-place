function [T, Ts] = POE(theta)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
%TODO: Add l1,l2,l3

if isa(theta, 'sym') 
    syms l1 l2 l3 l4;
    M = sym(eye(4));
    Ts = sym(zeros(4,4,4));

else
    l1 = 13.7; % IN CM
    l2 = 10.5;
    l3 = 10.5;
    l4 = 11.0;
    M = eye(4);
    Ts = zeros(4,4,4);
end



M(1:3,4) = [0; 0; l1+l2+l3+l4];

disp("My M is"); disp(M);

w1 = [0 0 1]';
w2 = [0 1 0]';
w3 = [0 1 0]'; 
w4 = [0 1 0]';

q1 = [0 0 0]';
q2 = [0 0 l1]';
q3 = [0 0 l1 + l2]';
q4 = [0 0 l1 + l2 + l3]';

S1 = [w1; cross(w1, -q1)];
S2 = [w2; cross(w2, -q2)];
S3 = [w3; cross(w3, -q3)];
S4 = [w4; cross(w4, -q4)];

Ts(:,:,1) = exp_mat_homogenous(S1, theta(1));
Ts(:,:,2) = exp_mat_homogenous(S2, theta(2));
Ts(:,:,3) = exp_mat_homogenous(S3, theta(3));
Ts(:,:,4) = exp_mat_homogenous(S4, theta(4));

T = Ts(:,:,1) * Ts(:,:,2) * Ts(:,:,3) * Ts(:,:,4) * M;

end