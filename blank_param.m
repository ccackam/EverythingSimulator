%% Header
% Parameters

% Add Paths
addpath ./../../

% Object for passing data
core = piping();

%% Pysical Parameters
param.k = ;                        % N/m - Spring Constant
param.m = ;                        % kg - Mass of Box
param.b = ;                      % N*sec/m - Damping constant

% System Limits
param.sat_lim.high = ;             % N - Upper limit of input
param.sat_lim.low = ;             % N - Lower Limit of input

% State Description
param.x_names = [];
param.u_names = [];

% Equlibrium
param.x_e = [];

% State Space Equations
param.A = [];
param.B = [];
param.C_r = [];
param.C_m = [];
param.D = [];

%% Uncertainty
% Here you are seting the standard deviation of the uncertianty
% Values next to each setting are useful for understanding the scale
% The values for random are one standard deviation of random error.
% The falues for bias are one standard deviations offset.
% k,m,b

param.uncertian_param = {};%{'k','m','b'};
param.D_in_param.random  = 0.2.*[];%[param.k,param.m,param.b];
param.D_in_param.bias    = 0.0.*[];%[param.k,param.m,param.b];
% F
param.uncertian_u = [false];
param.D_in_u.random      = [0.0];
param.D_in_u.bias        = [0.0]; 
% z,z_dot
param.uncertian_x = [false,false];
param.D_out.random       = [0,0];
param.D_out.bias         = [0,0];
% z,z_dot - measured
param.uncertain_N = [false,false];
param.N.random           = [0,0];
param.N.bias             = [0,0]; 

%% Simulation 
% Dimensions
settings.side = ;     % m - Dimension of the side of the box

% Simulation
settings.start       = 0;      % s
settings.step        = 0.01;   % s
settings.end         = 50;     % s
t = settings.start:settings.step:settings.end;

settings.publish     = 0.2;    % s
settings.window      = []; % m

%% Function Handles
functions.u_e = @u_e;
functions.eqs_motion = @eqs_motion;
functions.get_drawing = @get_drawing;

% Dynamic Equilibrium Input
function output = u_e(x,param)
    
end

% Equations of Motion
function x_dot = eqs_motion(t,x,u,param)

    % Initialize
    x_dot = zeros(length(x),1);

end

% Anamation Information
function [points,colors] = get_drawing(x,settings)
    points = {[]};
    colors = {};
end