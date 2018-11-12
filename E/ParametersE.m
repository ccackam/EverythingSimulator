%% Header
% Parameters

% Add Paths
addpath ./../../

%% Pysical Parameters
param.l = 0.5;      % m
param.m = [0.35,2]; % kg - 1=ball 2=beam
param.g = 9.81;      % m/s^2

% System Limits
param.sat_lim.high = 15;
param.sat_lim.low = -15;
param.hard_stop.high(1,1) = 0.5; 
param.hard_stop.high(2,1) = pi./2;
param.hard_stop.low(1,1) = 0.0;
param.hard_stop.low(2,1) = -pi./2;

% Equlibrium
param.x_e = [0.25;0;0;0];

% State Space Equations
l = param.l;
m = param.m;
g = param.g;
J = (1/3*l^2*m(2)+m(1)*(param.x_e(1))^2);
param.A = [0,   0,  1,  0;
           0,   0,  0,  1;
           0,  -g,  0,  0;
           g*m(1)/J,  0,  0,  0];
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
param.uncertian_u = [false];
param.D_in_u.random      = [0.0];
param.D_in_u.bias        = [0.5]; 
% z,z_dot
param.uncertian_x = [false,false,false,false];
param.D_out.random       = [0,0,0,0];
param.D_out.bias         = [0,0,0,0];
% z,z_dot - measured
param.uncertain_N = [false,false,false,false];
param.N.random           = [0.001,0.001,0,0];
param.N.bias             = [0,0,0,0]; 

%% Simulation 
% Dimensions
param.ball_r = 0.05;  % m
param.t = 0.01;      % m

% Simulation
sim.start       = 0;      % s
sim.step        = 0.01;   % s
sim.end         = 50;     % s
sim.publish     = 0.1;    % s
sim.window      = [0, 0.7, -0.6, 0.6]; % m

%% Functions
% Handles
param.control_architecture = @control_architecture;
param.u.e = @u_e;
param.eqs_motion = @eqs_motion;
param.get_drawing = @get_drawing;

% Control Architecture
function output = control_architecture(controllers,x,r,param)
    output = cascade(controllers,x,r);
end

% Dynamic Equilibrium Input
function output = u_e(x,param)
    output(1,1) = (param.m(2)*param.l./2 + param.m(1).*x(1)).*param.g./param.l;
end

% Equations of Motion
function x_dot = eqs_motion(t,x,u,param)
    %PROPAGATE Equations of motion. t is a dummy variable required
    %for ODE45. This method takes the current state in the form of
    %a vector containing x and x_dot. Then returns the x_dot and
    %x_ddot at that timestep.
    
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
function output = get_drawing(x,param)

    % Initial Drawing
    beam_points = [ 0,          0;
                    param.l,    0;
                    param.l,    -param.t;
                    0,          -param.t];
    ball_points = cos([0:0.01:1].'.*2.*pi).*param.ball_r;
    ball_points(:,2) = sin([0:0.01:1].'.*2.*pi).*param.ball_r;
            
    % Translate
    ball_points(:,1) = ball_points(:,1)+x(1);
    ball_points(:,2) = ball_points(:,2)+param.ball_r;
            
    % Rotate
    R = [cos(x(2)),-sin(x(2));
         sin(x(2)),cos(x(2))];
    beam_points = (R*beam_points.').';
    ball_points = (R*ball_points.').';

    points = {beam_points,ball_points};
    colors = {'k','b'};
    
    
    output = {points,colors};
end