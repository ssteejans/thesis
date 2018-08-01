%% Torque Requirements for Stepper Motors
% Sam Artho-Bentz

%% Clean up
clc
clear all
close all

%% Define System
F = 25;     % axial load (lbs)
dm = .330;  % mean diameter (inches)
f = .16;    % coeffcient of friction (bronze on steel)
l = 1/16;   % lead of screw (inches/revolution)

%% Calc
% Torque to raise load
Tr = F*dm/2*((l+pi*f*dm)/(pi*dm-f*l))   % (lb-in)

% Torque to lower load
Tl = F*dm/2*((-l+pi*f*dm)/(pi*dm+f*l))  % (lb-in)

disp(['The torque to raise the load is ' num2str(Tr) ' lb-in'])
disp(['The torque to raise the load is ' num2str(Tr*16) ' oz-in'])