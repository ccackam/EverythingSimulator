%% Header 

% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
addpath ./../
ParametersD;

%% Initial Conditions
x_0 = [0;0];
u_0 = functions.u_e(x_0,param);

%% Input Parameters
input       = 'square';  % Type of signal
period      = 100;        % Period of signal input
amplitude   = 0.5;        % Amplitude of signal input
offset      = 0;            % Offset from 0
phase_delay = 0;        % phase delay of function in rad
r = function_generator(input,period,amplitude,offset,phase_delay,t);

%% Simulation Parameters
settings.animation   = true;
settings.real_time   = false;
settings.plot_names  = {["z - Position (m)","z","z_{hat}","z_{r}"];
                        ["e - Observer Error","e_{z}","e_{z_{dot}}"];
                        ["F - Force (N)","F"]};

% General Variable to implement uncertainty
settings.implement_uncertainty = true;

% Display warnings?
warning('off','all')


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

t_r = 2.5;
zeta = 0.707;
p_i = -10;
p_d = -20;
control.controller_type = controllers.SS;

observe.type = observers.O; % O,e,p,m
observe.L = gains_O(t_r./10,zeta,p_d,param.A,param.B,param.C_m); % 0.05;
observe.x_names = ["z";"z_{dot}"];
observe.u_names = ["F"];
core.functions.observers(1) = observers(observe,core);

control.anti_windup = 'saturation'; % 'derivative', 'saturation', 'both', 'none'
control.windup_limit = 0.01;

if strcmp(control.controller_type,controllers.PID)
    % Z Position
    control.K.P = (2.2./t_r).^2.*param.m - param.k;
    control.K.I = 0.2;
    control.K.D = 2.*param.m.*zeta.*2.2./t_r - param.b;
    control.x_names = ["z";"z_{dot}"];
    control.u_names = ["F"];
    control.impose_sat = true;
    core.functions.controllers(1) = controllers(control,core);
elseif strcmp(control.controller_type,controllers.SS)
    % Z Position
    control.K = gains_SS(t_r,zeta,p_i,param.A,param.B,param.C_r);
    control.x_names = ["z";"z_{dot}"];
    control.u_names = ["F"];
    control.impose_sat = true;
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

function [L] = gains_O(t_r,zeta,p_d,A,B,C_m)

    w_n = 2.2./t_r;
    
    poles = [];
    for i = 1:length(t_r)
        Delta = [1,2.*zeta(i).*w_n(i),w_n(i).^2];
        poles = [poles;roots(Delta)];
    end
    
    if p_d ~= 0
        poles = [poles;p_d];
        A = [A,B;zeros([1,length(A)]),0];
        C_m = [C_m,zeros([length(C_m(:,1)),1])];
    end
    
    O_A = obsv(A,C_m);
    
    if rank(O_A) ~= length(A)
        error('System not observable!')
    end
    
    L.L = place(A',C_m',poles)';
    if p_d == 0
        L.d = zeros(size(L.L(1,:)));
    else
        L.d = L.L(end,:);
        L.L = L.L(1:end-1,:);
    end
end