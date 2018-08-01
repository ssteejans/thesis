%% Attempt to calculate telescope direction from actuator lengths
% Sam Artho-Bentz

% This will be used for determining an appropriate home position for the
% telescope. It will only be used in setting up the telescope.

%% Define Base Positions
syms Oz Ox Oy xs1 xh1 ys1 yh1 zh1 zs1 xs2 xh2 ys2 yh2 zh2 zs2 xs3 xh3 ys3 yh3 zh3 zs3

RotX = @(theta) [1,0,0;
        0, cos(theta), -sin(theta);
        0, sin(theta), cos(theta)];
RotY = @(theta) [cos(theta), 0, sin(theta);
       0, 1, 0;
       -sin(theta), 0, cos(theta)];
RotZ = @(theta) [cos(theta), -sin(theta), 0;
        sin(theta), cos(theta), 0;
        0,0,1];
    
    
Ps = [xs1, xs2, xs3; ys1, ys2, ys3; zs1, zs2, zs3];
Ph = [xh1, xh2, xh3; yh1, yh2, yh3; zh1, zh2, zh3];


sTh = RotZ(Oz)*RotX(-Ox)*RotY(-Oy);