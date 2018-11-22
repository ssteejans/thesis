%% Rotation Calculations
% Sam Artho-Bentz

%% Clean Up
clc
clear all
close all
format compact

%% Define desired angles
altAngles = [0:1:90];
azAngles = [-45:1:45];
rotAngle = 0;
lengthStorage = zeros(length(altAngles), length(azAngles), 6);
k = 0;
for i = 1:length(altAngles)
    for j = 1:length(azAngles)
        [lengthStorage(i,j,1), lengthStorage(i,j,2), lengthStorage(i,j,3), lengthStorage(i,j,4)]=calcLegLengths(altAngles(i), azAngles(j), rotAngle);
        lengthStorage(i,j,5) = altAngles(i); 
        lengthStorage(i,j,6) = azAngles(j);
        if lengthStorage(i,j,4) == 1
            k=k+1;
            dataout(k,:) = [altAngles(i), azAngles(j)];
        end
    end
end
polarplot(deg2rad(dataout(:,2)), dataout(:,1), '.b')