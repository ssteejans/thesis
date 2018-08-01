%% Rotation Calculations
% Sam Artho-Bentz

%% Clean Up
clc
clear all
close all
format compact

%% Define desired angles
altAngles = [90:-5:0];
azAngles = [-45:5:45];
rotAngle = 0;
lengthStorage = zeros(length(altAngles), length(azAngles), 6);
for i = 1:length(altAngles)
    for j = 1:length(azAngles)
        [lengthStorage(i,j,1), lengthStorage(i,j,2), lengthStorage(i,j,3), lengthStorage(i,j,4)]=calcLegLengths(altAngles(i), azAngles(j), rotAngle);
        lengthStorage(i,j,5) = altAngles(i); 
        lengthStorage(i,j,6) = azAngles(j);
    end
end
disp('done')