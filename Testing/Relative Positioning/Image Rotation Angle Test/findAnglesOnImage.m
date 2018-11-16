%% Line-Angle Calculation from Visual Inspection
% Sam Artho-Bentz
% 7/30/2018

%% Clean up from previous runs
clc
clear all
close all

%% Load the file
filename = 'IMG_7861.JPG';
imshow(filename)

%% Wait for user to draw a line which matches the angle of the negative line
h1=imline; %negative line
position1 = wait(h1);
angle1 = tan((position1(1,2)-position1(2,2))/(position1(1,1)-position1(2,1)));
close
imshow(filename)

%% Wait for user to draw a line which matches the angle of the neutral line
h2=imline; %zero line
position2 = wait(h2);
angle2 = tan((position2(1,2)-position2(2,2))/(position2(1,1)-position2(2,1)));
close
imshow(filename)

%% Wait for user to draw a line which matches the angle of the positive line
h3=imline; %positive line
position3 = wait(h3);
angle3 = tan((position3(1,2)-position3(2,2))/(position3(1,1)-position3(2,1)));
close

%% Output results
disp('corrected negative line is')
disp(angle1-angle2)
disp('corrected positive line is')
disp(angle3-angle2)
