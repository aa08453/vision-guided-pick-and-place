function w = skew_mat(w1, w2, w3)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
w = [0 -w3 w2;
     w3 0  -w1;
    -w2 w1 0];
end