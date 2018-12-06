%% Header

% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
addpath ./../
ParametersF;

%% Initial Conditions
param.x_0(1,1)    = 4;        % h - m
param.x_0(2,1)    = 0;        % z_v - m
param.x_0(3,1)    = 0;        % theta - degrees
param.x_0(4,1)    = 0;        % h_dot - m
param.x_0(5,1)    = 0;        % z_v_dot - m
param.x_0(6,1)    = 0;        % theta_dot - degrees
param.u_0(1,1)    = param.u.e(param.x_0,param)./2;        % fl - N
param.u_0(2,1)    = param.u.e(param.x_0,param)./2;        % fr - Nm
param.r_0(1,1)    = param.x_0(1);
param.r_0(2,1)    = 0;
param.r_0(3,1)    = 0;

%% Inputs
% Longitudinal
sim.input       = 'square';      % type of input
sim.period      = 100; %25;            % period of signal input
sim.amplitude   = 2.5;            % amplitude of signal input
sim.offset      = param.x_0(1);
sim.phase_delay = 0;
[r,~]           = function_generator(sim);
% Lateral
sim.input       = 'square';      % type of input
sim.period      = 100; %25;            % period of signal input
sim.amplitude   = 2.5;            % amplitude of signal input
sim.offset      = 0 ;
sim.phase_delay = 0.5; %0.25;
[r(:,2),t]           = function_generator(sim);

%% Simulation
sim.animation   = true;
sim.real_time   = false;    % Is the output simulation in real time? 
                            % or sped up?
sim.realistic   = false;      % Stop the simulation if the vtol crashes
sim.names = [   "h - Height (m)",
                "z_v - Position (m)",
                "f_l - Left Force (N)",
                "f_r - Reft Force (N)"];     % Names of data to display.
  % General Variable to implement uncertainty
param.implement_uncertainty = true;

% Display warnings?
warning('on','all')

            
%% Controller parameters
param.controller_type = controllers.SS;

if strcmp(param.controller_type,controllers.PID)

    % Longitudinal
    param.index = [1,0,0,1,0,0];
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

    % Lateral
    % Roll
    param.index = [0,1,0,0,1,0];
    param.t_r_z = 0.5;
    param.impose_sat = false;
    param.add_equilibrium = false;
    param.t_r = param.t_r_z.*4;
    param.zeta = 0.707;
    param.K.P = 2.2.^2./(-param.g.*param.t_r.^2);
    param.K.I = 0.001;
    param.K.D = (param.mu./sum(param.m) - 2.*param.zeta.*2.2./param.t_r)...
                        ./(param.g);
    param.sigma = 0.05;
    param.derivative_source = 'measure'; % 'measure', 'error', 'position'
    param.anti_windup = 'both'; % 'derivative', 'saturation', 'both', 'none'
    param.windup_limit = 0.05;
    param.controller(2) = controllers(param,sim);            

    % z
    param.index = [0,0,1,0,0,1];
    param.impose_sat = false;
    param.add_equilibrium = false;
    param.t_r = param.t_r_z;
    param.zeta = 0.707;
    param.K.P = (2.2./param.t_r).^2.*(param.Jc + 2.*param.m(1).*param.d.^2);
    param.K.I = 0.1;
    param.K.D = 2.*param.zeta.*2.2./param.t_r.*(param.Jc + 2.*param.m(1).*param.d.^2);
    param.sigma = 0.05;
    param.derivative_source = 'measure'; % 'measure', 'error', 'position'
    param.anti_windup = 'both'; % 'derivative', 'saturation', 'both', 'none'
    param.windup_limit = 0.05;
    param.controller(3) = controllers(param,sim); 
    
elseif strcmp(param.controller_type,controllers.SS)
    % Lon
    t_r = 1.75;
    zeta = 0.707;
    p_i = -5;
    param.K = gains_SS(t_r,zeta,p_i,param.A.lon,param.B.lon,param.C_r.lon);
    param.index = [1,0,0,1,0,0];
    param.impose_sat = false;
    param.add_equilibrium = true;
    param.sigma = 0.05;
    param.derivative_source = 'position'; % 'measure', 'error', 'position'
    param.anti_windup = 'both'; % 'derivative', 'saturation', 'both', 'none'
    param.windup_limit = 0.01;
    param.command_0 = param.r_0(1);
    param.controller(1) = controllers(param,sim);
    
    % Lat
    t_r_z = 0.5;
    t_r_roll = t_r_z.*4;
    t_r = [t_r_z,t_r_roll];
    zeta_z = 0.707;
    zeta_roll = 0.707;
    zeta = [zeta_z,zeta_roll];
    p_i = -5;
    param.K = gains_SS(t_r,zeta,p_i,param.A.lat,param.B.lat,param.C_r.lat);
    param.index = [0,1,1,0,1,1];
    param.impose_sat = false;
    param.add_equilibrium = false;
    param.sigma = 0.05;
    param.derivative_source = 'position'; % 'measure', 'error', 'position'
    param.anti_windup = 'both'; % 'derivative', 'saturation', 'both', 'none'
    param.windup_limit = 0.01;
    param.command_0 = param.r_0(2);
    param.controller(2) = controllers(param,sim);
end

%% Run Program
Simulation;