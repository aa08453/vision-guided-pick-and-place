function [T1, T2, T3, T4] = DH(theta,d, a, alpha)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


if isa(theta, 'sym') 
    T = sym(eye(4));
    Ts = sym(zeros(4,4,4));
else
    T = eye(4);
    Ts = zeros(4,4,4);

end


for i=1:length(theta)
    ct = cos(theta(i));
    ca = cos(alpha(i));
    st = sin(theta(i));
    sa = sin(alpha(i));
    T_ = [ct -st*ca st*sa a(i)*ct;
         st ct*ca -ct*sa a(i)*st;
         0 sa ca d(i);
         0 0 0 1];
    T = T*T_;
    Ts(:,:,i) = T;
end

T1 = Ts(:,:,1);
T2 = Ts(:,:,2);
T3 = Ts(:,:,3);
T4 = Ts(:,:,4);



end