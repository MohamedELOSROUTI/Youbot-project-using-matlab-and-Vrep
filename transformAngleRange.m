function tt = transformAngleRange(t, dt, r)
% TRANSFORMANGLERANGE
%   TT = TRANSFORMANGLERANGE(T, DT, R) adds DT to T and adapt it to new
%   angle range R (most common: [0 2*pi] and [-pi pi]).
    tt = t + dt;
    
    tt(tt > r(2)) = tt(tt > r(2)) - 2*pi;
    tt(tt < r(1)) = tt(tt < r(1)) + 2*pi;
end