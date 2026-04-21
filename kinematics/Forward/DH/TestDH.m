clear; clc; close all;
syms d1 d2 d3 d4 
syms alpha1 alpha2 alpha3 alpha4
syms a1 a2 a3 a4
syms theta1 theta2 theta3 theta4
arr = ["T01", "T12", "T23", "T34"];



syms l1 l2 l3 l4
d = [l1 0 0 0];
a = [0 l2 l3 14];
alpha = [sym(-pi/2) 0 0 0];



[T, Ts] = DH([theta1, theta2, theta3, theta4], ...
                        d,a,alpha);


for i=1:length(arr)
    disp(arr(i)); pretty(simplify(Ts(:,:,i)));
end
disp("Final is"); 
% pretty((simplify(expand(T))));
pretty(simplify(T));
disp("------------");

T01 = simplify(Ts(:,:,1));
disp("T01"); pretty(T01)

T14 = T01^-1 * T;
pretty(simplify(T14))
