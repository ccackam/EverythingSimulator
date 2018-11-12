% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
addpath ./../
ParametersD;

% Initial Conditions
param.x_0(1,1) = 0; % m - z
param.x_0(2,1) = 0; % m/s - z_dot
param.u_0(1,1) = param.u_e(param.x_0,param); % N - F
param.r_0(1,1) = 0; % m - r

% Input Parameters
sim.input       = 'square';  % Type of signal
sim.period      = 25;        % Period of signal input
sim.amplitude   = 1;        % Amplitude of signal input
sim.offset      = 0;            % Offset from 0
sim.phase_delay = 0;        % phase delay of function in rad

% Simulation Parameters
sim.real_time   = false;    % Is the output simulation in real time? 
                            % or sped up?
sim.realistic   = true;      % Stop the simulation if
sim.names       = ["z - Position (m)","F - Force (N)"];   % Names of data to display.
[r,t]           = function_generator(sim);

% Controller Parameters
param.index = 1;
param.impose_sat = false;
param.add_equilibrium = true;
param.controller_type = controllers.PD;
param.t_r = 2.15;
param.zeta = 0.707;
param.gains.K_P = (2.2./param.t_r).^2.*param.m - param.k;
param.gains.K_D = 2.*param.m.*param.zeta.*2.2./param.t_r - param.b;
param.controller(1) = controllers(param);

% General Variable to implement uncertainty
param.implement_uncertainty = true;

% Run Program
Simulation;