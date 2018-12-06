%% Header
% Parameters

% Add Paths
addpath ./../../

% Object for passing data
core = piping();

%% Pysical Parameters
param.l = 0.5;      % m
param.m = [0.35,2]; % kg - 1=ball 2=beam
param.g = 9.81;      % m/s^2

% System Limits
param.sat_lim.high = 15;
param.sat_lim.low = -15;

% State Description
param.x_names = ["z";"\theta";"z_{dot}";"\theta_{dot}"];
param.u_names = ["F"];

% Equlibrium
param.x_e = [param.l/2;0;0;0];

% % State Space Equations
l = param.l;
m = param.m;
g = param.g;
J = (1/3*l^2*m(2)+m(1)*(param.x_e(1))^2);
param.A = [0,   0,  1,  0;
           0,   0,  0,  1;
           0,  -g,  0,  0;
           -g*m(1)/J,  0,  0,  0];
param.B = [0;0;0;l./J];
param.C_r = [1,0,0,0];
param.C_m = [1,0,0,0;
             0,1,0,0];
param.D = [0;0];

%% Uncertainty
% Here you are seting the standard deviation of the uncertianty
% Values next to each setting are useful for understanding the scale
% The values for random are one standard deviation of random error.
% The falues for bias are one standard deviations offset.
% k,m,b

param.uncertian_param = {'m(1)','m(2)','l'};
param.D_in_param.random  = 0.2.*[param.m(1),param.m(2),param.l];
param.D_in_param.bias    = 0.0.*[param.m(1),param.m(2),param.l];
% F
param.uncertian_u = [true];
param.D_in_u.random      = [0.0];
param.D_in_u.bias        = [0.5]; 
% z,z_dot
param.uncertian_x = [false,false,false,false];
param.D_out.random       = [0,0,0,0];
param.D_out.bias         = [0,0,0,0];
% z,z_dot - measured
param.uncertain_N = [true,true,false,false];
param.N.random           = [0.001,0.001,0,0];
param.N.bias             = [0,0,0,0]; 

%% Simulation 
% Dimensions
settings.ball_r = 0.05;  % m
settings.t = 0.01;      % m

% Simulation
settings.start       = 0;      % s
settings.step        = 0.01;   % s
settings.end         = 50;     % s
t = settings.start:settings.step:settings.end;

settings.publish     = 0.2;    % s
settings.window      = [0, 0.7, -0.6, 0.6]; % m

%% Function Handles
functions.u_e = @u_e;
functions.eqs_motion = @eqs_motion;
functions.get_drawing = @get_drawing;

% Dynamic Equilibrium Input
function output = u_e(x,param)
    output(1,1) = (param.m(2)*param.l./2 + param.m(1).*x(1)).*param.g./param.l;
end

% Equations of Motion
function x_dot = eqs_motion(t,x,u,param)
    
    % Unpack
    F   = u;
    l   = param.l;
    g   = param.g;
    m1  = param.m(1);
    m2  = param.m(2);
    % --
    z           = x(1);
    theta       = x(2);
    z_dot       = x(3);
    theta_dot   = x(4);
    % --
    c = cos(theta);
    s = sin(theta);

    % Initialize
    x_dot = zeros(length(x),1);

    % Equations of Motion
    x_dot(1) = x(3);
    x_dot(2) = x(4);
    x_dot(3) = z*theta_dot^2 - g*s;
    x_dot(4) = (c*(F*l - m1*z*g - m2*g*l/2) - 2*m1*z*z_dot*theta_dot)...
                /(1/3*l^2*m2 + m1*z^2);

end

% Anamation Information
function [points,colors] = get_drawing(x,settings,param)
    % Initial Drawing
    beam_points = [ 0,          0;
                    param.l,    0;
                    param.l,    -settings.t;
                    0,          -settings.t];
    ball_points = cos([0:0.01:1].'.*2.*pi).*settings.ball_r;
    ball_points(:,2) = sin([0:0.01:1].'.*2.*pi).*settings.ball_r;
            
    % Translate
    ball_points(:,1) = ball_points(:,1)+x(1);
    ball_points(:,2) = ball_points(:,2)+settings.ball_r;
            
    % Rotate
    R = [cos(x(2)),-sin(x(2));
         sin(x(2)),cos(x(2))];
    beam_points = (R*beam_points.').';
    ball_points = (R*ball_points.').';

    points = {beam_points,ball_points};
    colors = {'k','b'};
end