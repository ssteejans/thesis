%% Rotation Calculations
% Find theoretically valid positions based on link lengths
% Sam Artho-Bentz

%% Clean Up
clc
clear all
%close all
format compact
%% Define Angles \theta
tic
for i = 1:180
    for j = 1:180
        for k = 1:20
    O_az(i,j,k) = deg2rad(i-91);     % Azimuth
    O_alt(i,j,k) = deg2rad(j-1);     % Altitude
    O_rot(i,j,k) = deg2rad(k);     % Rotation


    %% Define Base Positions
    % All subjective directions (left/right) based on an observer operating the telescope
    P0_b = [0,0,0]';    % Origin with respect to the base
    P1_b = [-12,-2.7,16]';    % Right Vertical with respect to the base
    P2_b = [12,-2.7,16]';    % Left Vertical with respect to the base
    P3_b = [-12,-2.7,5]';    % Horizontal with respect to the base
    OA_b = P0_b;            %Optical Axis base
    %% Define Home Positions
    P0_h = [0,0,0]'-P0_b;
    P1_h = [-13.75,6.75,10.6875]';
    P2_h = [6,6.4375,16.6875]';    
    P3_h = [-1.2,-3.75,4]';  
    OA_h = [-6, 0, 19.875]';

    %% Calculate Correction Angles \phi to Rotate from home position to alt = 0, az = 0 
    phi_az = -atan2(OA_h(1), OA_h(3));
    phi_alt = -atan2(OA_h(2), OA_h(3));

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
    sTb = RotZ(O_rot(i,j,k))*RotX(-O_alt(i,j,k))*RotX(phi_alt)*RotY(-O_az(i,j,k))*RotY(phi_az);

    %% Rotate
    P0_r = sTb*P0_h;
    P1_r = sTb*P1_h;
    P2_r = sTb*P2_h;
    P3_r = sTb*P3_h;
    OA_r = sTb*OA_h;


    L_p1 = norm(P1_r-P1_b);
    L_p2 = norm(P2_r-P2_b);
    L_p3 = norm(P3_r-P3_b);

    if (L_p1<11)||(L_p2<11)||(L_p3<11)
        %disp('This is not a valid input')
        result(i,j,k)=0;
    else
        result(i,j,k)=1;
    end
    end
    end
end
toc

