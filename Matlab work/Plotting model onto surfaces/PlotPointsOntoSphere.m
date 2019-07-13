%% Rotation Calculations
% Sam Artho-Bentz


%% Clean Up
clc
clear all
close all
format compact

%% Define desired angles
altAngles = [0:.1:90]; %[10:0.1:20];
azAngles = [-15:5:15]; %[-70:1:45];
rotAngle = 0;
pointStorage = zeros(length(altAngles), length(azAngles), 6);
r = 1;

for j = 1:length(azAngles)
    for i = 1:length(altAngles)
        pointStorage(i,j,1) = altAngles(i);
        pointStorage(i,j,2) = azAngles(j);
        pointStorage(i,j,3) = rotAngle;
        pointStorage(i,j,4:6) = findPoints(pointStorage(i,j,1), pointStorage(i,j,2), pointStorage(i,j,3));
        %% Apply Intersection Equation
        OA_r = pointStorage(i,j,4:6);
        x1 = OA_r(1);
        y1 = OA_r(2);
        z1 = OA_r(3);
        t = sqrt(4*(x1^2+y1^2+z1^2)*r^2)/(2*(x1^2+y1^2+z1^2));
        point(i,j, 1) = x1*t;
        point(i,j, 2) = y1*t;
        point(i,j, 3) = z1*t;
    end
end

%% Plot
figure(1)
sphere(50)
alpha('clear')
hold on
axis equal
for j = 1:length(azAngles)
    plot3(point(:,j,1), point(:,j,2), point(:,j,3))
    pause(.001)
end
xlim([-1 1])
ylim([0 1])
zlim([0 1])
view(0,0)


function [OA_r] = findPoints(O_alt, O_az, O_rot)

O_alt = deg2rad(O_alt);
O_az = deg2rad(O_az);
O_rot = deg2rad(O_rot);

%% Define Base Positions
% All subjective directions (left/right) based on an observer operating the telescope
P0_b = [0,0,0]';                    % Origin with respect to the base (Ball Joint)
P1_b = [-12,-3.25,16]';             % Right Vertical with respect to the base
P2_b = [12,-3.25,16]';              % Left Vertical with respect to the base
P3_b = [-12,-3.25,5]';              % Horizontal with respect to the base
OA_b = P0_b;                        % Optical Axis base

%% Define Home Positions
P0_h = [0,0,0]'-P0_b;               % ???
P1_h = [-13.55    8.55 10.64]';     % Nut on Actuator One in the Home position
P2_h = [6.85    8.87   15.63]';     % Nut on Actuator Two in the Home position
P3_h = [-1.32   -2.44    5.57]';    % Nut on Actuator Three in the Home position
OA_h = [-4.0   3.75   16.25]';      % Location of the far end of the Optical Axis in the Home position
P1P2_length = norm(P1_h-P2_h);



%% Calculate Allowable Lengths
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

% syms Oz Ox Oy
% sTbsyms = RotZ(Oz)*RotX(Ox)*RotY(Oy);

%% Rotate
P0_r = sTb*P0_h;
P1_r = sTb*P1_h;
P2_r = sTb*P2_h;
P3_r = sTb*P3_h;
OA_r = sTb*OA_h;



end