%% Header
% Parameters

% Add Paths
addpath ./../../

% Object for passing data
core = piping();

%% Pysical Parameters
param.d = 0.3;              % m
param.l  = 0.5;             % m
param.m  = [0.25,1,0.25];   % kg - 1=left motor 2=centner 3=right motor
param.Jc = 0.0042;          % kg*m^s
param.g  = 9.81;            % m/s^2
param.mu = 0.1;
param.M = sum(param.m);
param.J = (param.Jc + 2.*param.m(1).*param.d.^2);

% System Limits
param.sat_lim.high = 20;             % N - Upper limit of input
param.sat_lim.low = 0;             % N - Lower Limit of input

% State Description
param.x_names = ["h";"z_{v}";"\theta";"h_{dot}";"z_{v}_{dot}";"\theta_{dot}"];
param.u_names = ["F";"\tou"];

% Equlibrium
param.x_e = [4;0;0;0;0;0];

% State Space Equations
d = param.d;
l = param.l;
m = param.m;
Jc = param.Jc;
g = param.g;
mu = param.mu;
M = param.M;
J = param.J;
Fe = u_e(param.x_e,param);
param.A = [0,   0,   0,  1,  0,  0;
           0,   0,   0,  0,  1,  0;
           0,   0,   0,  0,  0,  1;
           0,   0,   0,  0,  0,  0;
           0,   0,   -Fe(1)/M,0,mu/M,0;
           0,   0,   0,  0,  0,  0];
param.B = [0,0;0,0;0,0;1/M,0;0,0;0,1./J];
param.C_r = [1,0,0,0,0,0;
             0,1,0,0,0,0];
param.C_m = [1,0,0,0,0,0;
             0,1,0,0,0,0;
             0,0,1,0,0,0];
param.D = [0,0;0,0;0,0];

%% Uncertainty
param.uncertian_param = {'m(2)','Jc','d','mu'};
param.D_in_param.random  = 0.2.*[param.m(2),param.Jc,param.d,param.mu];
param.D_in_param.bias    = 0.0.*[param.m(2),param.Jc,param.d,param.mu];
% F
param.uncertian_u = [true,true];
param.D_in_u.random      = [0.0,0.0];
param.D_in_u.bias        = [1.0*sum(param.m),1.0*sum(param.m)]; 
% z,z_dot
param.uncertian_x = [false,false,false,false,false,false];
param.D_out.random       = [0,0,0,0,0,0];
param.D_out.bias         = [0,0,0,0,1.0,0];
% z,z_dot - measured
param.uncertain_N = [false,false,false,false,false,false];
param.N.random           = [0,0,0,0,0,0];
param.N.bias             = [0,0,0,0,0,0];

%% Simulation 
% Dimensions
settings.propX = param.d./2;   % m
settings.propY = param.d./8;   % m
settings.cargo = param.d./4;   % m
settings.side = 0.1;           % m

% Simulation
settings.start       = 0;      % s
settings.step        = 0.01;   % s
settings.end         = 50;     % s
t = settings.start:settings.step:settings.end;

settings.publish     = 0.2;    % s
settings.window      = [-4, 4, 0, 8]; % m

%% Function Handles
functions.u_e = @u_e;
functions.eqs_motion = @eqs_motion;
functions.get_drawing = @get_drawing;

% Dynamic Equilibrium Input
function output = u_e(x,param)
    output(1,1) = param.g.*sum(param.m);
    output(2,1) = 0;
end

% Equations of Motion
function x_dot = eqs_motion(t,x,u,param)

    % Unpack
    h = x(1);
    z_v = x(2);
    theta = x(3);
    h_dot = x(4);
    z_v_dot = x(5);
    theta_dot = x(6);
    %---
    M = param.M;
    J = param.J;
    mu = param.mu;
    d = param.d;
    g = param.g;
    %---
    F = u(1);
    Tau = u(2);
    %---
    s = sin(theta);
    c = cos(theta);

    % Initialize
    x_dot = zeros(6,1);

    % Equations of Motion
    x_dot(1) = h_dot;
    x_dot(2) = z_v_dot;
    x_dot(3) = theta_dot;
    x_dot(4) = c*F./M - g;
    x_dot(5) = -(s*F + mu*z_v_dot)/M;
    x_dot(6) = Tau./J;

end

% Anamation Information
function [points,colors] = get_drawing(x,settings,param)
    % Build VTOL
    propL_points = [settings.propX.*cos([0:0.1:1].'.*2.*pi)-param.d,...
                    settings.propY.*sin([0:0.1:1].'.*2.*pi)];
    cargo_points = [-settings.cargo./2,0;
                    -settings.cargo./2,settings.cargo./2;
                    settings.cargo./2,settings.cargo./2;
                    settings.cargo./2,0;
                    settings.cargo./2,0;
                    settings.cargo./2,-settings.cargo./2;
                    -settings.cargo./2,-settings.cargo./2;
                    -settings.cargo./2,0];
    propR_points = [settings.propX.*cos([-1:0.2:1].'.*pi)+param.d,...
                    settings.propY.*sin([-1:0.2:1].'.*pi)];
    vtol_origin = [propL_points;
                          cargo_points(1:4,:)
                          propR_points;
                          cargo_points(5:8,:);
                          propL_points(1,:)];

    % Rotate
    theta = x(3);
    R = [cos(theta),-sin(theta);
         sin(theta),cos(theta)];
    vtol_origin = (R*(vtol_origin.')).';

    % Translate
    vtol_origin(:,1) = vtol_origin(:,1) + x(2);
    vtol_origin(:,2) = vtol_origin(:,2) + x(1);

    % Pack
    points = {vtol_origin};
    colors = {'b'};
end