%% Header
% Parameters

% Add Paths
addpath ./../../

%% Pysical Parameters
param.k = 3;                        % N/m - Spring Constant
param.m = 5;                        % kg - Mass of Box
param.b = 0.5;                      % N*sec/m - Damping constant

% System Limits
param.sat_lim.high = 2;             % N - Upper limit of input
param.sat_lim.low = -2;             % N - Lower Limit of input
param.hard_stop.high(1,1) = inf;    % m - Upper limit of x
param.hard_stop.low(1,1) = -inf;    % m - Lower limit of x

% Equlibrium
param.x_e = [0;0];

% State Space Equations
param.A = [0, 1;
     -param.k./param.m, -param.b./param.m];
param.B = [0;1./param.m];
param.C_r = [1,0];
param.C_m = [1,0];
param.D = [0];

%% Uncertainty
% Here you are seting the standard deviation of the uncertianty
% Values next to each setting are useful for understanding the scale
% The values for random are one standard deviation of random error.
% The falues for bias are one standard deviations offset.
% k,m,b
param.uncertian_param = {'k','m','b'};
param.D_in_param.random  = 0.2.*[param.k,param.m,param.b];
param.D_in_param.bias    = 0.0.*[param.k,param.m,param.b];
% F
param.uncertian_u = [false];
param.D_in_u.random      = [0.0];
param.D_in_u.bias        = [0.5]; 
% z,z_dot
param.uncertian_x = [false,false];
param.D_out.random       = [0,0];
param.D_out.bias         = [0,0];
% z,z_dot - measured
param.uncertain_N = [false,false];
param.N.random           = [0.001,0];
param.N.bias             = [0,0]; 

%% Simulation 
% Dimensions
param.side = 0.25;     % m - Dimension of the side of the box

% Simulation
sim.start       = 0;      % s
sim.step        = 0.01;   % s
sim.end         = 50;     % s
sim.publish     = 0.1;    % s
sim.window      = [-1.5,1.5,0,0.75]; % m

%% Function Handles
param.control_architecture = @control_architecture;
param.u.e = @u_e;
param.eqs_motion = @eqs_motion;
param.get_drawing = @get_drawing;

% Control Architecture
function output = control_architecture(controller,x,r,param)
    u = controller.master(x,r);
    output = {u,r};
end

% Dynamic Equilibrium Input
function output = u_e(x,param)
    output(1,1) = x(1,1).*param.k;
end

% Equations of Motion
function x_dot = eqs_motion(t,x,u,param)
    %PROPAGATE Equations of motion. t is a dummy variable required
    %for ODE45. This method takes the current state in the form of
    %a vector containing x and x_dot. Then returns the x_dot and
    %x_ddot at that timestep.

    % Initialize
    x_dot = zeros(length(x),1);

    % Equations of Motion
    x_dot(1) = x(2);
    x_dot(2) = (u(1) - param.b.*x(2) - param.k.*x(1))./param.m; 
end

% Anamation Information
function output = get_drawing(x,param)
    points = {[  x(1),0;
                x(1) + param.side,0;
                x(1) + param.side,param.side;
                x(1),param.side;
                x(1),0]};
    colors = {'g'};
    
    
    output = {points,colors};
end

