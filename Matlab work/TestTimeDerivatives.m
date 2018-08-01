
close all

syms O_x O_y w_x w_y w_z
RotX = @(theta) [1,0,0;
        0, cos(theta), -sin(theta);
        0, sin(theta), cos(theta)];
RotY = @(theta) [cos(theta), 0, sin(theta);
       0, 1, 0;
       -sin(theta), 0, cos(theta)];
RotZ = @(theta) [cos(theta), -sin(theta), 0;
        sin(theta), cos(theta), 0;
        0,0,1];
TimeDerivator = [0, -w_z, w_y; w_z, 0 -w_x; -w_y, w_x, 0];
Transformed = RotX(O_x)*RotY(O_y)*RotY(1)
TimeDerivator*Transformed