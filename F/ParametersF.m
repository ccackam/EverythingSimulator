%% Header
% Parameters

% Add Paths
addpath ./../../

%% Pysical Parameters
param.d = 0.3;              % m
param.l  = 0.5;             % m
param.m  = [0.25,1,0.25];   % kg - 1=left motor 2=centner 3=right motor
param.Jc = 0.0042;          % kg*m^s
param.g  = 9.81;            % m/s^2
param.mu = 0.1;             % kg/s - The airflow through the rotor 
                            % creates a change in the direction of 
                            % flow of air and causes what is called 
                            % "momentum drag." Momentum drag can be 
                            % modeled as a viscous drag force that 
                            % is proportional to the horizontal velocity 
                            % z_v. In other words, the drag force is 
                            % F_drag = mu*z_dot
param.M = sum(param.m);
param.J = (param.Jc + 2.*param.m(1).*param.d.^2);
                            
% System Limits
param.sat_lim.high = 20;
param.sat_lim.low = 0;
param.hard_stop.high(1,1) = inf; 
param.hard_stop.high(2,1) = inf;
param.hard_stop.high(3,1) = inf;
param.hard_stop.low(1,1) = 0;
param.hard_stop.low(2,1) = -inf;
param.hard_stop.low(3,1) = -inf;

% Equilibrium
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
Fe = u_e(x_e,param);
% lon
param.A.lon = [0, 1;
               0, 0];
param.B.lon = [0;1./M];
param.C_r.lon = [1,0];
param.C_m.lon = [1,0];
param.D.lon = [0];

% lat
param.A.lat = [0,   0,  1,  0;
               0,   0,  0,  1;
               0,  -Fe/M,  mu/M,  0;
               0,  0,  0,  0];
param.B.lat = [0;0;0;1./J];
param.C_r.lat = [1,0,0,0];
param.C_m.lat = [1,0,0,0;
                 0,1,0,0];
param.D.lat = [0;0];



%% Uncertainty
% Here you are seting the standard deviation of the uncertianty
% Values next to each setting are useful for understanding the scale
% The values for random are one standard deviation of random error.
% The falues for bias are one standard deviations offset.
% k,m,b
param.uncertian_param = {'m(2)','Jc','d','mu'};
param.D_in_param.random  = 0.2.*[param.m(2),param.Jc,param.d,param.mu];
param.D_in_param.bias    = 0.0.*[param.m(2),param.Jc,param.d,param.mu];
% F
param.uncertian_u = [false,false];
param.D_in_u.random      = [0.0,0.0];
param.D_in_u.bias        = [1.0*sum(param.m),0.0]; 
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
param.propX = param.d./2;   % m
param.propY = param.d./8;   % m
param.cargo = param.d./4;   % m
param.side = 0.1;           % m

% Simulation
sim.start       = 0;        % s
sim.step        = 0.01;     % s
sim.end         = 30;       % s
sim.publish     = 0.2;      % s
sim.window      = [-4, 4, 0, 8]; % m

%% Function Handles
param.control_architecture = @control_architecture;
param.u.e = @u_e;
param.eqs_motion = @eqs_motion;
param.get_drawing = @get_drawing;

% Control Architecture
function output = control_architecture(controllers,x,r,param)

    u_long = controllers(1).master(x,r(1,:));
    r_long = r(1,:);
    
    output_lat = cascade(controllers(2:end),x,r(2,:));
    u_lat = output_lat{1};
    r_lat = output_lat{2};
    
    u(1,1) = u_long./2 - u_lat./(2.*param.d);
    u(2,1) = u_long./2 + u_lat./(2.*param.d);
    r = [r_long;r_lat];
    
    output = {u,r};
end

% Dynamic Equilibrium Input
function output = u_e(x,param)
    output(1,1) = param.g.*sum(param.m);
end

% Equations of Motion
function x_dot = eqs_motion(t,x,u,param)
    %PROPAGATE Equations of motion. t is a dummy variable required
    %for ODE45. This method takes the current state in the form of
    %a vector containing x and x_dot. Then returns the x_dot and
    %x_ddot at that timestep.
    
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
    fl = u(1);
    fr = u(2);
    F = fr + fl;
    Tau = d.*fr - d.*fl;
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
function output = get_drawing(x,param)

    % Build VTOL
    propL_points = [param.propX.*cos([0:0.1:1].'.*2.*pi)-param.d,...
                    param.propY.*sin([0:0.1:1].'.*2.*pi)];
    cargo_points = [-param.cargo./2,0;
                    -param.cargo./2,param.cargo./2;
                    param.cargo./2,param.cargo./2;
                    param.cargo./2,0;
                    param.cargo./2,0;
                    param.cargo./2,-param.cargo./2;
                    -param.cargo./2,-param.cargo./2;
                    -param.cargo./2,0];
    propR_points = [param.propX.*cos([-1:0.2:1].'.*pi)+param.d,...
                    param.propY.*sin([-1:0.2:1].'.*pi)];
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

    points = {vtol_origin};
    colors = {'b'};
    
    
    output = {points,colors};
end