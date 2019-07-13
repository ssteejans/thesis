%% Rotation Calculations
% Sam Artho-Bentz


%% Clean Up
clc
clear all
close all
format compact


%% Define Geometry





%% Define desired angles
altAngles = [15]; %[10:0.1:20];
azAngles = [15]; %[-70:1:45];
rotAngle = 0;
lengthStorage = zeros(length(altAngles), length(azAngles), 6);
k = 0;

for i = 1:length(altAngles)
    for j = 1:length(azAngles)
        [lengthStorage(i,j,1), lengthStorage(i,j,2), lengthStorage(i,j,3), lengthStorage(i,j,4)]=testValidity(altAngles(i), azAngles(j), rotAngle);
        lengthStorage(i,j,5) = altAngles(i); 
        lengthStorage(i,j,6) = azAngles(j);
        if lengthStorage(i,j,4) == 1
            k=k+1;
            dataout(k,:) = [altAngles(i), azAngles(j)];
        end
        if lengthStorage(i,j,4) == 0
            k=k+1;
            datanot(k,:) = [altAngles(i), azAngles(j)];
        end
    end
end

polarplot(deg2rad(dataout(:,2)), dataout(:,1), '.b')
figure()
hold on
plot(dataout(:,1), dataout(:,2), '.b')
plot(datanot(:,1), datanot(:,2), '.r')
xlim([altAngles(1) altAngles(end)])
ylim([azAngles(1) azAngles(end)])
xlabel('Altitude')
ylabel('Azimuth')
hold off

function [ L_p1, L_p2, L_p3, valid ] = testValidity(O_alt, O_az, O_rot)

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

%% Define Maximum Leg Lengths
% Measured beyond the Home Position
L_p1max = 24;
L_p2max = 24;
L_p3max = 10;

%% Define Center of Mass
CM_h = [-1.91, 5.74, 7.42]';         % Center of Mass in the Home Position

%% Define invalid region for test of perpendicularity of front legs
perpDeviation = 0.1;                  % How close the legs can get to 90 degrees (5 means 85-95 degrees is rejected)

%% Calculate Allowable Lengths
L_p1min = rssq(P1_h-P1_b);
L_p2min = rssq(P2_h-P2_b);
L_p3min = rssq(P3_h-P3_b);

L_p1max = L_p1min + L_p1max;
L_p2max = L_p2min + L_p2max;
L_p3max = L_p3min + L_p3max;
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
P0_r = sTb*P0_h
P1_r = sTb*P1_h
P2_r = sTb*P2_h
P3_r = sTb*P3_h
OA_r = sTb*OA_h
CM_r = sTb*CM_h

L_p1 = norm(P1_r-P1_b);
L_p2 = norm(P2_r-P2_b);
L_p3 = norm(P3_r-P3_b);


%% Test for validity of points

% Check that leg lengths are between min and max
testLength = (L_p1<L_p1min)||(L_p2<L_p2min)||(L_p3<L_p3min)||(L_p1>L_p1max)||(L_p2>L_p2max)||(L_p3>L_p3max);

% Check that Center of Mass is within the X,Z boundary formed by the two front legs and the origin  
testCM = ~inpolygon(CM_r(1), CM_r(3), [P0_h(1), P1_h(1), P2_h(1)], [P0_h(3), P1_h(3), P2_h(3)]); 

% Test that the front legs are not perpendicular to the base
% find vector of each front leg and a vector parallel to the base
line1 = P1_r-P1_b;
line2 = P2_r-P2_b;
lineBase = P1_b-P2_b;

% normalize the vectors to length zero
line1Norm = line1/norm(line1);
line2Norm = line2/norm(line2);
lineBaseNorm = lineBase/norm(lineBase);

% use cos(theta) = dot(vector1, vector2)/(norm(vector1)*norm(vector2))
% norm of all vectors is 1 so that can be excluded
% arccos of each side leaves radians
% change to degrees and subtract from 90 to find distance from perp
testPerp1 = abs(90-rad2deg(acos(dot(line1Norm, lineBaseNorm))));
testPerp2 = abs(90-rad2deg(acos(dot(line2Norm, lineBaseNorm))));

% Check test results
if testLength||testCM||testPerp1<perpDeviation||testPerp2<perpDeviation
    valid = 0;
else
    valid = 1;
end

end