%% Header

% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
addpath ./../
Parameters_Midterm;

%% Initial Conditions
param.x_0(1,1)    = 0;        % x2-x1
param.x_0(2,1)    = 0;        % x1
param.x_0(3,1)    = 0;        % x2dot-x1dot 
param.x_0(4,1)    = 0;        % x1dot
param.u_0(1,1)    = 0;        % f
param.r_0(1,1)    = 0;

%% Inputs
sim.input       = 'square';      % type of input
sim.period      = 1/0.05; %25;            % period of signal input
sim.amplitude   = 2.5;            % amplitude of signal input
sim.offset      = 2.5;
sim.phase_delay = 0;

sim.input       = 'square';      % type of input
sim.period      = 1/0.05; %25;            % period of signal input
sim.amplitude   = 2.5;            % amplitude of signal input
sim.offset      = 2.5;
sim.phase_delay = 0;

[r,t]           = function_generator(sim);

%% Simulation
sim.animation   = true;
sim.real_time   = false;    % Is the output simulation in real time? 
                            % or sped up?
sim.realistic   = false;      % Stop the simulation if the vtol crashes
sim.names = [   "z_3 - Separation (m)",
                "f - Input Force (N)"];     % Names of data to display.
  % General Variable to implement uncertainty
param.implement_uncertainty = false;

% Display warnings?
warning('on','all')

            
%% Controller parameters
param.controller_type = controllers.SS;

if strcmp(param.controller_type,controllers.PID)
    % Longitudinal
    param.index = [1,0,1,0];
    param.impose_sat = false;
    param.add_equilibrium = true;
    param.t_r = 1.75;
    param.zeta = 0.707;
    param.K.P = (2.2./param.t_r).^2.*sum(param.m);
    param.K.I = 0.25;
    param.K.D = 2.*param.zeta.*2.2./param.t_r.*sum(param.m);
    param.sigma = 0.05;
    param.derivative_source = 'measure'; % 'measure', 'error', 'position'
    param.anti_windup = 'both'; % 'derivative', 'saturation', 'both', 'none'
    param.windup_limit = 0.05;
    param.controller(1) = controllers(param,sim);
    
elseif strcmp(param.controller_type,controllers.SS)
    % No information is given regarding saturation limits. So I cranked up
    % (or i guess techinically down) the rise time to get a very fast
    % responce.
    t_r_z1 = 0.2;
    t_r_z2 = t_r_z1.*1.1;
    t_r = [t_r_z1,t_r_z2];
    zeta_z1 = 0.707;
    zeta_z2 = 0.707;
    zeta = [zeta_z1,zeta_z2];
    p_i = 0;
    param.K = gains_SS(t_r,zeta,p_i,param.A,param.B,param.C_r);
    param.index = [1,1,1,1];
    param.impose_sat = false;
    param.add_equilibrium = false;
    param.sigma = 0.05;
    param.derivative_source = 'measure'; % 'measure', 'error', 'position'
    param.anti_windup = 'none'; % 'derivative', 'saturation', 'both', 'none'
    param.windup_limit = 0.01;
    param.command_0 = param.r_0(1);
    param.controller(1) = controllers(param,sim);
    
end

%% Run Program
Simulation;