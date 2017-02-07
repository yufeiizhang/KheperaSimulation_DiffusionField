% Main.m for simulation
clear

%% Load Data
% load field data
load('field.mat'); % previous 1750points.mat
% load robot trajectory

%% Global Parameter
% load parameter file Scr_para
Scr_para;
% explicit parameter settings
%% in Scr_trig
% maximum loop number 
loopNumMax = 10;
%% in Scr_frame
% frame number for initialize CO2 field in experiment, skip in simulation
skipInitField = 120;
% field concentration offset for simulation, some cases the fitting result may lower than 0
concOffset = 100;

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