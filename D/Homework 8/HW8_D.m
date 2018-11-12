%% Header 

% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
addpath ./../
ParametersD;

%% Initial Conditions
param.x_0(1,1) = 0; % m - z
param.x_0(2,1) = 0; % m/s - z_dot
param.u_0(1,1) = param.u.e(param.x_0,param); % N - F
param.r_0(1,1) = 0; % m - r

%% Input Parameters
sim.input       = 'square';  % Type of signal
sim.period      = 100;        % Period of signal input
sim.amplitude   = 0.5;        % Amplitude of signal input
sim.offset      = 0;            % Offset from 0
sim.phase_delay = 0;        % phase delay of function in rad
[r,t]           = function_generator(sim);

%% Simulation Parameters
sim.animation   = false;
sim.real_time   = false;    % Is the output simulation in real time? 
                            % or sped up?
sim.realistic   = true;      % Stop the simulation if
sim.names       = ["z - Position (m)","F - Force (N)"];   % Names of data to display.

% General Variable to implement uncertainty
param.implement_uncertainty = false;

% Display warnings?
warning('on','all')

%% Controller Parameters
param.controller_type = controllers.SS;

if strcmp(param.controller_type,controllers.PID)
    % Z Position
    t_r = 2.0;
    zeta = 0.707;
    param.K.P = (2.2./t_r).^2.*param.m - param.k;
    param.K.I = 0;%0.2;
    param.K.D = 2.*param.m.*zeta.*2.2./t_r - param.b;
    param.index = [1,1];
    param.impose_sat = false;
    param.add_equilibrium = true;
    param.sigma = 0.05;
    param.derivative_source = 'position'; % 'measure', 'error', 'position'
    param.anti_windup = 'both'; % 'derivative', 'saturation', 'both', 'none'
    param.windup_limit = 0.01;
    param.controller(param.index) = controllers(param,sim);
elseif strcmp(param.controller_type,controllers.SS)
    % Z Position
    t_r = 2.0;
    zeta = 0.707;
    [param.K.K,param.K.k_r] = gains_SS(t_r,zeta,param.A,param.B,param.C_r);
    param.index = [1,1];
    param.impose_sat = false;
    param.add_equilibrium = true;
    param.sigma = 0.05;
    param.derivative_source = 'measure'; % 'measure', 'error', 'position'
    param.anti_windup = 'both'; % 'derivative', 'saturation', 'both', 'none'
    param.windup_limit = 0.01;
    param.controller(param.index) = controllers(param,sim);
end


%% Run Program
Simulation;

%% Functions

% function [num,den] = find_tf(A,B,C,D)
%     C
% end