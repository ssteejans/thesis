function [ L_p1, L_p2, L_p3, valid ] = calcLegLengths(O_alt, O_az, O_rot)

O_alt = deg2rad(O_alt);
O_az = deg2rad(O_az);
O_rot = deg2rad(O_rot);
%% Define Base Positions
% All subjective directions (left/right) based on an observer operating the telescope
P0_b = [0,0,0]';    % Origin with respect to the base
P1_b = [-12,-3.25,16]';    % Right Vertical with respect to the base
P2_b = [12,-3.25,16]';    % Left Vertical with respect to the base
P3_b = [-12,-3.25,5]';    % Horizontal with respect to the base
OA_b = P0_b;            %Optical Axis base

%% Define Home Positions
P0_h = [0,0,0]'-P0_b;
P1_h = [-13.55    8.55 10.64]';
P2_h = [6.85    8.87   15.63]';    
P3_h = [-1.32   -2.44    5.57]';  
OA_h = [-4.0   3.75   16.25]';
P1P2_length = norm(P1_h-P2_h);

%% Calculate Minimum Lengths
L_p1min = rssq(P1_h-P1_b);
L_p2min = rssq(P2_h-P2_b);
L_p3min = rssq(P3_h-P3_b);

%% Calculate Correction Angles \phi to Rotate from home position to alt = 0, az = 0 
phi_az = -atan2(OA_h(1), OA_h(3));
phi_alt = atan2(OA_h(2), OA_h(3));
phi_rot = atan2(P1_h(2)-P2_h(2), P1P2_length);
%% Define Rotation Matrices and Translation Matrix

RotX = @(theta) [1,0,0;
        0, cos(theta), -sin(theta);
        0, sin(theta), cos(theta)];
RotY = @(theta) [cos(theta), 0, sin(theta);
       0, 1, 0;
       -sin(theta), 0, cos(theta)];
RotZ = @(theta) [cos(theta), -sin(theta), 0;
        sin(theta), cos(theta), 0;
        0,0,1];
    
%% Assemble Translation Matrix from Base(b) to Telescope(s)
% Rotations Azimuth->Altitude->Image Rotation
sTb = RotZ(O_rot+phi_rot)*RotX(-O_alt+phi_alt)*RotY(-O_az+phi_az);

syms Oz Ox Oy

sTbsyms = RotZ(Oz)*RotX(Ox)*RotY(Oy);
%% Rotate
P0_r = sTb*P0_h;
P1_r = sTb*P1_h;
P2_r = sTb*P2_h;
P3_r = sTb*P3_h;
OA_r = sTb*OA_h;


L_p1 = norm(P1_r-P1_b);
L_p2 = norm(P2_r-P2_b);
L_p3 = norm(P3_r-P3_b);

if (L_p1<L_p1min)||(L_p2<L_p2min)||(L_p3<L_p3min)
    valid = 0;
   % disp('This is not a valid input')
else
    valid = 1;
end

end

