%% Rotation Calculations
% Sam Artho-Bentz

%% Clean Up
clc
clear all
close all
format compact
%% Define Angles \theta

O_alt = .1%deg2rad(30);     % Altitude
O_az = .05%deg2rad(10);     % Azimuth
O_rot = 0%deg2rad(0);     % Rotation - Positive is Clockwise from perspective 
                                % of observer operating telescope
w_alt = .1;              % angular velocity of altitude rad/sec
w_az = .05;               % angular velocity of azimuth rad/sec
w_rot = 0;              %angular velocity of image rotation rad/sec

%% Define Base Positions
% All subjective directions (left/right) based on an observer operating the telescope
P0_b = [0,0,0]';    % Origin with respect to the base
P1_b = [-12,-4.75,16]';    % Right Vertical with respect to the base
P2_b = [12,-4.75,16]';    % Left Vertical with respect to the base
P3_b = [-12,-4.75,5]';    % Horizontal with respect to the base
OA_b = P0_b;            %Optical Axis base

%% Define Home Positions
P0_h = [0,0,0]'-P0_b;
P1_h = [-14.1875    4.4375   12.3438]';
P2_h = [6.3125    2.8750   18.2500]';    
P3_h = [-1.3750   -4.0000    7.0625]';  
OA_h = [-4.6250   -1.7500   15.7500]';
P1P2_length = norm(P1_h-P2_h);
%% Calculate Correction Angles \phi to Rotate from home position to alt = 0, az = 0 
phi_az = -atan2(OA_h(1), OA_h(3));
phi_alt = -atan2(OA_h(2), OA_h(3));
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
RotXDot = @(theta, omega) [0, 0, 0; 0, 0, -omega; 0, omega, 0]*RotX(theta)
RotYDot = @(theta, omega) [0, 0, omega; 0, 0, 0; -omega, 0, 0]*RotY(theta)
RotZDot = @(theta, omega) [0, -omega, 0; omega, 0, 0; 0, 0, 0]*RotZ(theta)

%% Assemble Translation Matrix from Base(b) to Telescope(s)
% Rotations Azimuth->Altitude->Image Rotation
sTb = RotZ(O_rot+phi_rot)*RotX(-O_alt+phi_alt)*RotY(-O_az+phi_az);

%% Rotate
P0_r = sTb*P0_h;
P1_r = sTb*P1_h;
P2_r = sTb*P2_h;
P3_r = sTb*P3_h;
OA_r = sTb*OA_h;


L_p1 = norm(P1_r-P1_b)
L_p2 = norm(P2_r-P2_b)
L_p3 = norm(P3_r-P3_b)

if (L_p1<11)||(L_p2<11)||(L_p3<11)
    disp('This is not a valid input')
end

VelRotation1 = [0, -w_rot, w_az; w_rot, 0, -w_alt; -w_az, w_alt, 0]*sTb;
VelRotation2 = RotZDot(O_rot+phi_rot, w_rot)*RotX(-O_alt+phi_alt)*RotY(-O_az+phi_az)+...
                RotZ(O_rot+phi_rot)*RotXDot(-O_alt+phi_alt,w_alt)*RotY(-O_az+phi_az)+...
                RotZ(O_rot+phi_rot)*RotX(-O_alt+phi_alt)*RotYDot(-O_az+phi_az, w_az);
V1_r = VelRotation2*P1_h;
V2_r = VelRotation2*P2_h;
V3_r = VelRotation2*P3_h;
L_p1_dot = norm(V1_r)
L_p2_dot = norm(V2_r)
L_p3_dot = norm(V3_r)