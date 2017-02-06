% Main.m for simulation
clear

%% Load Data
% load field data
load('field.mat'); % previous 1750points.mat
% load robot trajectory

%% Global Parameter
% load parameter file Scr_para
Scr_para;

%% Initial System
% load initial settings Scr_init
Scr_init;
counter = 2; % reset counter

%% Loop
while(1)
    % update loop counter
    counter = counter + 1;
    % Scr_frame
    Scr_frame;
    % Scr_trig
    Scr_trig;
end

%% Clean Up
Scr_clean;