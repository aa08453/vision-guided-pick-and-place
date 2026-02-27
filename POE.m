function T = POE(theta)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
%TODO: Add l1,l2,l3
l1 = 1;
l2 = 1;
l3 = 1;

M = eye(4);
M(3,3) = l1+l2+l3;

w1 = [ 0 0 1 ]';
w2 = [ 1 0 0]';
w3 = [1 0 0]'; 

q1 = [0 0 l1]';
q2 = [0 0 l1 + l2]';
q3 = [0 0 l1 + l2 + l3]';

S1 = [w1; cross(w1, -q1)];
S2 = [w2; cross(w2, -q2)];
S3 = [w3; cross(w3, -q3)];

exp1 = exp_mat_homogenous(S1, theta(1));
exp2 = exp_mat_homogenous(S2, theta(2));
exp3 = exp_mat_homogenous(S3, theta(3));

T = exp1 * exp2 * exp3 * M;

end