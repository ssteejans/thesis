%% Tuning a Stepper Motor for use with the L6470
% Sam Artho-Bentz
% 4/10/2017
% based on ST AN4144

%% Clean Up
clc
clear all
close all

%% Define system
Rm = 21.6;      % Resistance of one phase [ohm]
Lm = 100e-3;    % Inductance of one phase [H]
ke = (36.88/2)/93.9; % motor electrical constant. Found via method on pg 14 [V/Hz]
Vbus = 10;      % supply/operating voltage [V]
Iph = .45;      % target current per phase [A]

%% Calculate Parameters
Kval = round(Rm*abs(Iph)/Vbus*2^8);
INT_SPEED = round(4*Rm/(2*pi*Lm)*2^26*250e-9);
ST_SLP = round((ke/4)/Vbus*2^16);
FN_SLP = round(((2*pi*Lm*abs(Iph)+ke)/4)/Vbus*2^16);

%% Check Max
if INT_SPEED > 2^14
    INT_SPEED = 2^14;
end
if Kval > 2^8
    disp('Kval too high, inappropriate system characteristics')
elseif ST_SLP > 2^8
    disp('ST_SLP too high, inappropriate system characteristics')
elseif FN_SLP > 2^8
    disp('FN_SLP too high, inappropriate system characteristics')
else
    disp(['KVAL = ' num2str(Kval) '(' dec2bin(Kval,8) ')'])
    
    disp(['INT_SPEED = ' num2str(INT_SPEED) '(' dec2bin(INT_SPEED,14) ')'])
    
    disp(['ST_SLP = ' num2str(ST_SLP) '(' dec2bin(ST_SLP,8) ')'])
    
    disp(['FN_SLP = ' num2str(FN_SLP) '(' dec2bin(FN_SLP,8) ')'])
end