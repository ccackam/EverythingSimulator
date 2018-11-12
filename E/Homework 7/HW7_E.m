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
param.u_0(1,1) = param.u.e(param.x_0,param); % N - F
param.r_0(1,1) = param.l./2;           % m - z
param.r_0(2,1) = 0;           % rad - theta

% Input Parameters
sim.input       = 'square';  % Type of input
sim.period      = 25;        % Period of signal input
sim.amplitude   = 0.15;     % Amplitude of signal input
sim.offset      = 0.25;     % offset from 0
sim.phase_delay = 0;        % phase delay of function

sim.input       = 'other';  % Type of signal
sim.amplitude   = 0;        % Amplitude of signal input
sim.offset      = 0.4;      % Offset from 0
[r,t]           = function_generator(sim);

% Simulation Parameters
sim.real_time   = false;     % Is the output simulation in real time? 
                            % or sped up?                            
sim.realistic   = false;      % Stop the simulation if the ball falls off
sim.names = [   "z - Position Along Beam (m)",...
                "\theta - Angle (rad)",...
                "F - Force (N)"];     % Names of data to display.

% Controller parameters
param.index = 1;
param.t_r_inner = 0.14;
param.impose_sat = false;
param.add_equilibrium = false;
param.controller_type = controllers.PID;
param.t_r = param.t_r_inner.*8;
param.zeta = 0.707;
param.K.P = 2.2.^2./(-param.g.*param.t_r.^2);
param.K.I = -0.1;
param.K.D = 2.*param.zeta.*2.2./(-param.g.*param.t_r);
param.sigma = 0.05;
param.derivative_source = 'position'; % 'measure', 'error', 'position'
param.anti_windup = 'both'; % 'derivative', 'saturation', 'both', 'none'
param.windup_limit = 0.01;
param.controller(1) = controllers(param,sim);

param.index = 2;
param.impose_sat = true;
param.add_equilibrium = true;
param.controller_type = controllers.PID;
param.t_r = param.t_r_inner;
param.zeta = 0.707;
param.b_0 = param.l./(param.m(2)*param.l.^2./3 + param.m(1).*(param.l./2).^2);
param.K.P = 2.2.^2./(param.b_0.*param.t_r.^2);
param.K.I = 0.0;
param.K.D = 2.*param.zeta.*2.2./(param.b_0.*param.t_r);
param.sigma = 0.05;
param.derivative_source = 'measure'; % 'measure', 'error', 'position'
param.anti_windup = 'both'; % 'derivative', 'saturation', 'both', 'none'
param.windup_limit = 0.05;
param.controller(2) = controllers(param,sim);

% General Variable to implement uncertainty
param.implement_uncertainty = true;

% Display warnings?
warning('on','all')

% Run Program
Simulation;