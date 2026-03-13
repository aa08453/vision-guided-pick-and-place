function T = DH(theta,d, a, alpha)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
T = eye(4);
for i=length(theta)

ct = cos(theta(i));
ca = cos(alpha(i));
st = sin(theta(i));
sa = sin(alpha(i));
T_ = [ct -st*ca st*sa a(i)*ct;
     st ct*ca -ct*sa a(i)*st;
     0 sa ca d(i);
     0 0 0 1];
T = T*T_;
end


end