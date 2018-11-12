% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
addpath ./../
ParametersE;

% Initial Conditions
param.initial(1) = 0;           % theta - degrees
param.initial(2) = param.l./2;  % r - m
param.initial(3) = 0;           % theta_dot - degrees/s
param.initial(4) = 0;           % r_dot - m/s

% sum(param.m)./2.*param.g

% Simulation Parameters
sim.input       = 'sine';  % Type of input
sim.period      = 200;        % Period of signal input
sim.amplitude   = 10;     % Amplitude of signal input
sim.real_time   = true;     % Is the output simulation in real time? 
                            % or sped up?
sim.realistic = false;      % Stop the simulation if the ball falls off
sim.names = ["Theta - Angle (rad)",...
    "r - Position Along Beam (m)"];     % Names of data to display.
sim.initial = param.initial(1:2);       % Initial condition of the 
                                        % function generator.