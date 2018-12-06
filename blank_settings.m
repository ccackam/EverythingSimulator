%% Header 

% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
addpath ./../
warning('Do not forget parameters file')

%% Initial Conditions
x_0 = ;
u_0 = functions.u_e(x_0,param);

%% Input Parameters
input       = ;  % Type of signal
period      = ;        % Period of signal input
amplitude   = ;        % Amplitude of signal input
offset      = ;            % Offset from 0
phase_delay = ;        % phase delay of function in rad
r = function_generator(input,period,amplitude,offset,phase_delay,t);

%% Simulation Parameters
settings.animation   = true;
settings.real_time   = false;
settings.plot_names  = {[];
                        [];
                        []};

% General Variable to implement uncertainty
settings.implement_uncertainty = ;

% Display warnings?
warning('on','all')


%% Publish Data
core.publish("x",x_0)
core.publish("x_hat",x_0)
core.publish("u",u_0)
core.publish_list("r",r)
core.publish_list("t",t)
core.settings = settings;
core.param = param;
core.functions = functions;

%% Controller Parameters

t_r = ;
zeta = 0.707;
p_i = ;
control.controller_type = controllers.SS;

observe.type = observers.; % O,e,p,m
observe.L = gains_O(t_r./,zeta,param.A,param.C_m); % 0.05;
observe.x_names = [];
observe.u_names = [];
core.functions.observers(1) = observers(observe,core);

control.anti_windup = ; % 'derivative', 'saturation', 'both', 'none'
control.windup_limit = 0.01;

if strcmp(control.controller_type,controllers.PID)
    % Z Position
    control.K.P = ;
    control.K.I = ;
    control.K.D = ;
    control.x_names = [];
    control.u_names = [];
    core.functions.controllers(1) = controllers(control,core);
elseif strcmp(control.controller_type,controllers.SS)
    % Z Position
    control.K = gains_SS(t_r,zeta,p_i,param.A,param.B,param.C_r);
    control.x_names = [];
    control.u_names = [];
    core.functions.controllers(1) = controllers(control,core);
end

%% Run Program
Simulation;

%% Calculate Gains
function [K] = gains_SS(t_r,zeta,p_i,A,B,C_r)

    w_n = 2.2./t_r;
    
    poles = [];
    for i = 1:length(t_r)
        Delta = [1,2.*zeta(i).*w_n(i),w_n(i).^2];
        poles = [poles;roots(Delta)];
    end
    
    if p_i ~= 0
        poles = [poles;p_i];
        A = [A,zeros(length(A),1);-C_r,0];
        B = [B;0];
    end
    
    C_A = ctrb(A,B);
    
    if rank(C_A) ~= length(A)
        error('System not controlable!')
    end
    
    K.K = place(A,B,poles);
    
    if p_i == 0
        K.k_r = -1./(C_r*((A-B*K.K)^-1)*B);
        K.I = 0;
    elseif p_i ~= 0
        K.k_r = 0;
        K.I = K.K(end);
        K.K = K.K(1:end-1);
    end
    
end

function [L] = gains_O(t_r,zeta,A,C_m)

    w_n = 2.2./t_r;
    
    poles = [];
    for i = 1:length(t_r)
        Delta = [1,2.*zeta(i).*w_n(i),w_n(i).^2];
        poles = [poles;roots(Delta)];
    end
    
    O_A = obsv(A,C_m);
    
    if rank(O_A) ~= length(A)
        error('System not observable!')
    end
    
    L = place(A',C_m',poles)';
    
end