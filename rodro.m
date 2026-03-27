function exp_mat = rodro(omega, theta)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
omega_mat = skew_mat(omega(1), omega(2), omega(3));
exp_mat = eye(3) + sin(theta).*omega_mat + cos(theta).*(omega_mat^2);
end