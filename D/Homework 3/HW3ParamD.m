% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
addpath ./../
ParametersD;

% Initial Conditions
param.initial(1) = 0; % m - z
param.initial(2) = 0; % m/s - z_dot
param.initial(3) = 0; % m/s^2 - z_ddot
param.initial(4) = 0; % N - F

% Simulation Parameters
sim.input       = 'sine';  % Type of signal
sim.period      = 2.*pi;        % Period of signal input
sim.amplitude   = 9;        % Amplitude of signal input
sim.real_time   = false;    % Is the output simulation in real time? 
                            % or sped up?
sim.names       = "z - Position (m)";   % Names of data to display.
sim.initial     = param.initial(1);     % Initial condition of the function generator.