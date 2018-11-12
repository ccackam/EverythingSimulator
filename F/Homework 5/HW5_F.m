% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
addpath ./../
ParametersF;

% Initial Conditions
param.x_0(1)    = 4;        % h - m
param.x_0(2)    = 0;        % z_v - m
param.x_0(3)    = 0;        % theta - degrees
param.x_0(4)    = 0;        % h_dot - m
param.x_0(5)    = 0;        % z_v_dot - m
param.x_0(6)    = 0;        % theta_dot - degrees
param.u_0(1)    = param.u_e(param.x_0,param)./2;        % fl - N
param.u_0(2)    = param.u_e(param.x_0,param)./2;        % fr - Nm
param.r_0(1)    = param.x_0(1);
param.r_0(2)    = 0;
param.r_0(3)    = 0;

% Inputs
% Longitudinal
sim.input       = 'square';      % type of input
sim.period      = 25;            % period of signal input
sim.amplitude   = 2.5;            % amplitude of signal input
sim.offset      = param.x_0(1);
sim.phase_delay = 0;
[r,~]           = function_generator(sim);
% Lateral
sim.input       = 'square';      % type of input
sim.period      = 25;            % period of signal input
sim.amplitude   = 2.5;            % amplitude of signal input
sim.offset      = 0 ;
sim.phase_delay = 0.25;
[r(:,2),t]           = function_generator(sim);

% Simulation
sim.real_time   = false;    % Is the output simulation in real time? 
                            % or sped up?
sim.realistic   = false;      % Stop the simulation if the vtol crashes
sim.names = [   "h - Height (m)",
                "z_v - Position (m)",
                "\theta - Orientation (rad)",
                "f_l - Left Force (N)",
                "f_r - Reft Force (N)"];     % Names of data to display.

            
% Controller parameters
% Longitudinal
param.index = 1;
param.impose_sat = false;
param.add_equilibrium = true;
param.controller_type = controllers.PD;
param.t_r = 4;
param.zeta = 0.707;
param.gains.K_P = (2.2./param.t_r).^2.*param.M;
param.gains.K_D = 2.*param.zeta.*2.2./param.t_r.*param.M;
param.controller(1) = controllers(param);

% Lateral
param.index = 2;
param.t_r_inner = 0.8;
param.impose_sat = false;
param.add_equilibrium = false;
param.controller_type = controllers.PD;
param.t_r = param.t_r_inner.*4;
param.zeta = 0.707;
param.gains.K_P = 2.2.^2./(-param.g.*param.t_r.^2);
param.gains.K_D = (param.mu./param.M - 2.*param.zeta.*2.2./param.t_r)...
                    ./(param.g);
param.controller(2) = controllers(param);            

param.index = 3;
param.impose_sat = false;
param.add_equilibrium = false;
param.controller_type = controllers.PD;
param.t_r = param.t_r_inner;
param.zeta = 0.707;
param.gains.K_P = (2.2./param.t_r).^2.*param.J;
param.gains.K_D = 2.*param.zeta.*2.2./param.t_r.*param.J;
param.controller(3) = controllers(param); 

% Run Program
Simulation;