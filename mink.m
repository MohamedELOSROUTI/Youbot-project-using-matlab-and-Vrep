function [ M, I ] = mink( A, k )
%MINK Summary of this function goes here
%   Detailed explanation goes here

    M = zeros([1 k]); I = zeros([1 k]);
    temp = A;
    for i=1:k
        [m, in] = min(temp);
        M(i) = m; I(i) = in;
        
        temp(in) = inf;
    end

end

