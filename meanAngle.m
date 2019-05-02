function [ mean_agl ] = meanAngle( agls, wgt )
%MEANANGLE Summary of this function goes here
%   Detailed explanation goes here
    
    if isempty(wgt)
        wgt = ones(size(agls));
    end
    mean_agl = atan2( sum(sin(agls).*wgt)/sum(wgt), sum(cos(agls).*wgt)/sum(wgt) );

end

