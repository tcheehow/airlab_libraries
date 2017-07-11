function [ alpha, dX ] = PosEstimator( ra1, ra3 )
%POSESTIMATOR Summary of this function goes here
%   Detailed explanation goes here
    A = [ra1 -1;
        ra3 1];
    R = 2.5;
    B = R * [1;1];
    X = A\B;
    offset = pi/4;
    alpha = acos(X(1)) - offset;
    dX = X(2);
end

