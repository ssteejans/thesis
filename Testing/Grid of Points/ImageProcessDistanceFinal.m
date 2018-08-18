%% Final Assignment (8): Image Processing
% Samuel S. Artho-Bentz


%% Clean Up
clc
clear all
close all

%% Input
myPhoto = imread('picture.jpg');
redstats=myImgStats(myPhoto,1);
greenstats=myImgStats(myPhoto,2);
bluestats=myImgStats(myPhoto,3);

% %% with geometry!
% % Initial Test using Geometry. Results in dist = 20.076in
% distance1=(((bluestats(1).Centroid(1)-bluestats(2).Centroid(1))^2+(bluestats(1).Centroid(2)-bluestats(2).Centroid(2))^2)^.5)/12
% distance2=(((bluestats(2).Centroid(1)-bluestats(3).Centroid(1))^2+(bluestats(2).Centroid(2)-bluestats(3).Centroid(2))^2)^.5)/12
% distance3=(((bluestats(1).Centroid(1)-greenstats(1).Centroid(1))^2+(bluestats(1).Centroid(2)-greenstats(1).Centroid(2))^2)^.5)/18
% distance4=(((bluestats(3).Centroid(1)-greenstats(2).Centroid(1))^2+(bluestats(3).Centroid(2)-greenstats(2).Centroid(2))^2)^.5)/18
% 
% pxperin=mean([distance1 distance2 distance3 distance4])
% 
% reddist=(((redstats(1).Centroid(1)-redstats(2).Centroid(1))^2+(redstats(1).Centroid(2)-redstats(2).Centroid(2))^2)^.5)/pxperin

%% linreg vert
x=[bluestats(1).Centroid(2)-bluestats(2).Centroid(2), greenstats(1).Centroid(2)-bluestats(2).Centroid(2)];
y=[12, 30];
vertreg=myLinReg(x,y);
dr1y=vertreg(1)*(redstats(1).Centroid(2)-bluestats(2).Centroid(2))+vertreg(2);
dr2y=vertreg(1)*(redstats(2).Centroid(2)-bluestats(2).Centroid(2))+vertreg(2);

%% linreg horizontal
x=[bluestats(3).Centroid(1)-bluestats(2).Centroid(1), greenstats(2).Centroid(1)-bluestats(2).Centroid(1)];
y=[12, 30];
vertreg=myLinReg(x,y);
dr1x=vertreg(1)*(redstats(1).Centroid(1)-bluestats(2).Centroid(1))+vertreg(2);
dr2x=vertreg(1)*(redstats(2).Centroid(1)-bluestats(2).Centroid(1))+vertreg(2);

%% distances 
% Results dist = 19.435in
ydist=dr1y-dr2y;
xdist=dr1x-dr2x;
totdist=hypot(ydist, xdist)