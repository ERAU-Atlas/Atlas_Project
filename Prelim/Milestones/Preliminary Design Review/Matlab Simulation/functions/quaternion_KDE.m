function [dot_q] = quaternion_KDE(w, q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

dot_q = .5*[   0,   -w(1), -w(2), -w(3);
            w(1),       0,  w(3), -w(2);
            w(2),   -w(3),     0,  w(1);
            w(3),    w(2), -w(1),     0] * q;

end