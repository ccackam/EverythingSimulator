%% Header
% Parameters

% Add Paths
addpath ./../../

%% Pysical Parameters
param.m(1) = 2500; % kg
param.m(2) = 320;  % kg
param.k(1) = 80000;  % N/m
param.k(2) = 500000; % N/m
param.b(1) = 350;    % N-s/m
param.b(2) = 15020;  % N-s/m
                            
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
param.x_e = [0;0;0;0];

% State Space Equations
m1 = param.m(1); % kg
m2 = param.m(2);  % kg
k1 = param.k(1);  % N/m
k2 = param.k(2); % N/m
b1 = param.b(1);    % N-s/m
b2 = param.b(2);  % N-s/m
param.Fe = u_e(param.x_e,param);
% Note that I adjusted these matricies to match my code. (i.e. I changed the order of x)
param.A = [-((b1/m1)+(b1/m2)+(b2/m2)),                   b2/m2,               1,           0;
           0,                                            0,                   0,           1;
           -((k1/m1)+(k1/m2)+(k2/m2)),                   k2/m2,               0,           0;
           ((b1/m1)*((b1/m1)+(b1/m2)+(b2/m2)))-(k1/m1),  -(b1*b2)/(m1*m2),    -(b1/m1),    0];
param.B=[0; 
         1/m1;
         0;
         (1/m1)+(1/m2)];
param.B = [0;
           0;
           (1/m1)+(1/m2);
           1/m1];
           
param.C_r = [1,0,0,0];
param.C_m = [1,1,1,1];
param.D = [0];



%% Uncertainty
% Here you are seting the standard deviation of the uncertianty
% Values next to each setting are useful for understanding the scale
% The values for random are one standard deviation of random error.
% The falues for bias are one standard deviations offset.
% k,m,b
param.uncertian_param = {};
param.D_in_param.random  = 0.2.*[];
param.D_in_param.bias    = 0.0.*[];
% F
param.uncertian_u = [false,true];
param.D_in_u.random      = [0.0,0.0];
param.D_in_u.bias        = [1.0*sum(param.m),0.1]; 
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
param.w = 5;
param.h = 2;
param.d = 5;

% Simulation
sim.start       = 0;        % s
sim.step        = 0.01;     % s
sim.end         = 60;       % s
sim.publish     = 0.2;      % s
sim.window      = [-5,5,0,25]; % m

%% Function Handles
param.control_architecture = @control_architecture;
param.u.e = @u_e;
param.eqs_motion = @eqs_motion;
param.get_drawing = @get_drawing;

% Control Architecture
function u = control_architecture(controllers,x,r,param)
    u = cascade(controllers,x,r);
end

% Dynamic Equilibrium Input
function output = u_e(x,param)
    output(1,1) = 0;
end

% Equations of Motion
function x_dot = eqs_motion(t,x,u,param)

    % Equations of Motion
    x_dot = param.A*x + param.B*u;
    
end

% Anamation Information
function output = get_drawing(x,param)
        first_mass = [-param.w./2,param.d-param.h/2;
                      param.w./2,param.d-param.h/2;
                      param.w./2,param.d+param.h/2;
                      -param.w./2,param.d+param.h/2];
        second_mass = [-param.w./2,param.d*2-param.h/2;
                      param.w./2,param.d*2-param.h/2;
                      param.w./2,param.d*2+param.h/2;
                      -param.w./2,param.d*2+param.h/2];
        first_mass(:,2) = first_mass(:,2) + x(2);
        second_mass(:,2) = second_mass(:,2) + x(1) + x(2);
        points = {first_mass,second_mass};
        colors = {'b','b'};
        
        output = {points,colors};
end