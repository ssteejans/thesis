%% Calculate Home Geometry
% Requires x, z locations of each actuator in the home position
% as well as the length of the acutators. Any length unit is acceptable but
% must be consistent
% Sam Artho-Bentz
% 4/25/2017

%% Clean Up
clc
clear all
close all

%% Base Positions
P1_b = [-12,-4.75,16];   % Right Vertical with respect to the base
P2_b = [12,-4.75,16];    % Left Vertical with respect to the base
P3_b = [-12,-4.75,5];    % Horizontal with respect to the base

%% Measured Lengths
L1 = 5.375+4.75;
L2 = 5.375+4.75;
L3 = 5.375+5.25;

%% Measurement Position offset
% Easier to measure from the edge of board for example
Offx = 24.5;
Offy = 7.5;
Offz = 3;

%% X, Y, Z positions of each actuator
P1x = (11.25+9+3/8)/2-Offx;
P2x = (6+6+5/8)/2;
P3x = (-1/8-2-5/8)/2;
P1z = (16+5/8+14+1/16)/2-Offz;
P2z = (19+15/16+22+9/16)/2-Offz;
P3z = (6+3/4+4+3/4+2+5/8)/2;
P1y = (12.125+11.75)/2-Offy;
P2y = (11.25+9.5)/2-Offy;
P3y = (3+5/8+3+3/8)/2-Offy;

%% X, Y, Z of point on OTA
OAx = -4-5/8 ;
OAy = 5.75   -Offy;
OAz = 18.75 -Offz;

%% Assemble Points
P1_h = [P1x, P1y, P1z]
P2_h = [P2x, P2y, P2z]
P3_h = [P3x, P3y, P3z]
OA_h = [OAx, OAy, OAz]
%% Check against measured length
tol = (L1+L2+L3)/3*.05;
if abs(L1-norm(P1_h-P1_b))>tol || abs(L2-norm(P2_h-P2_b))>tol || abs(L3-norm(P3_h-P3_b)) >tol
    disp('Out of tolerance. Check measurements')
end