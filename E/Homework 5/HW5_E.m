% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
addpath ./../
ParametersE;

% Initial Conditions
param.x_0(1,1) = param.l./2;     % z - degrees
param.x_0(2,1) = 0;           % theta - m
param.x_0(3,1) = 0;           % z_dot - degrees/s
param.x_0(4,1) = 0;           % theta - m/s
param.u_0(1,1) = param.u_e(param.x_0,param); % N - F
param.r_0(1,1) = param.l./2;           % m - z
param.r_0(2,1) = 0;           % rad - theta

% Input Parameters
sim.input       = 'square';  % Type of input
sim.period      = 25;        % Period of signal input
sim.amplitude   = 0.15;     % Amplitude of signal input
sim.offset      = 0.25;     % offset from 0
sim.phase_delay = 0;        % phase delay of function

% Simulation Parameters
sim.real_time   = false;     % Is the output simulation in real time? 
                            % or sped up?                            
sim.realistic   = false;      % Stop the simulation if the ball falls off
sim.names = [   "z - Position Along Beam (m)",...
                "\theta - Angle (rad)",...
                "F - Force (N)"];     % Names of data to display.
[r,t]           = function_generator(sim);

% Controller parameters
param.index = 1;
param.t_r_inner = 1;
param.impose_sat = false;
param.add_equilibrium = false;
param.controller_type = controllers.PD;
param.t_r = param.t_r_inner.*4;
param.zeta = 0.707;
param.gains.K_P = 2.2.^2./(-param.g.*param.t_r.^2);
param.gains.K_D = 2.*param.zeta.*2.2./(-param.g.*param.t_r);
param.controller(1) = controllers(param);

param.index = 2;
param.impose_sat = false;
param.add_equilibrium = true;
param.controller_type = controllers.PD;
param.t_r = param.t_r_inner;
param.zeta = 0.707;
param.b_0 = param.l./(param.m(2)*param.l.^2./3 + param.m(1).*(param.l./2).^2);
param.gains.K_P = 2.2.^2./(param.b_0.*param.t_r.^2);
param.gains.K_D = 2.*param.zeta.*2.2./(param.b_0.*param.t_r);
param.controller(2) = controllers(param);

% Run Program
Simulation;