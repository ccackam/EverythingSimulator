%% Header 

% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
addpath ./../
ParametersF;

%% Initial Conditions
x_0 = [4;0;0;0;0;0];
u_0 = functions.u_e(x_0,param);

%% Input Parameters
% Longitudinal
input       = 'square';  % Type of signal
period      = 100;        % Period of signal input
amplitude   = 2.5;        % Amplitude of signal input
offset      = x_0(1);            % Offset from 0
phase_delay = 0;        % phase delay of function in rad
r = function_generator(input,period,amplitude,offset,phase_delay,t);
% Lateral
input       = 'square';  % Type of signal
period      = 100;        % Period of signal input
amplitude   = 2.5;        % Amplitude of signal input
offset      = 0;            % Offset from 0
phase_delay = 0.25;        % phase delay of function in rad
r(2,:) = function_generator(input,period,amplitude,offset,phase_delay,t);



%% Simulation Parameters
settings.animation   = true;
settings.real_time   = false;
settings.plot_names  = {["h - Height (m)","h","h_{hat}","h_{r}"];
                        ["z_{v} - Position (m)","z_{v}","z_{v}_{hat}","z_{v}_{r}"];
                        ["e - Observer Error","e_{h}","e_{z_{v}}","e_{\theta}","e_{h_{dot}}","e_{z_{v}_{dot}}","e_{\theta_{dot}}"];
                        ["F - Forces (N)","F","\tou"]};

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

t_r_lon = 2.2;
zeta_lon = 0.707;
p_i_lon = -5;
p_d_lon = -10;
t_r_lat_theta = 0.1644;
zeta_lat_theta = 0.707;
t_r_lat_z = t_r_lat_theta*13.5;
zeta_lat_z = 0.707;
p_i_lat = -5;
p_d_lat = -10;
t_r_lat = [t_r_lat_z,t_r_lat_theta];
zeta_lat = [zeta_lat_z,zeta_lat_theta];
control.controller_type = controllers.SS;

observe.type = observers.O; % O,e,p,m
observe.L = gains_O(t_r_lon./5,zeta_lon,p_d_lon,param.A([1,4],[1,4]),param.B([1,4],1),param.C_m(1,[1,4])); % 0.05;
observe.x_names = ["h";"h_{dot}"];
observe.u_names = ["F"];
core.functions.observers(1) = observers(observe,core);

observe.type = observers.O; % O,e,p,m
observe.L = gains_O(t_r_lat./5,zeta_lat,p_d_lat,param.A([2,3,5,6],[2,3,5,6]),param.B([2,3,5,6],2),param.C_m([2,3],[2,3,5,6])); % 0.05;
observe.x_names = ["z_{v}";"\theta";"z_{v}_{dot}";"\theta_{dot}"];
observe.u_names = ["\tou"];
core.functions.observers(2) = observers(observe,core);

control.anti_windup = 'none'; % 'derivative', 'saturation', 'both', 'none'
control.windup_limit = 0.01;

if strcmp(control.controller_type,controllers.PID)
    % h
    control.K.P = (2.2./param.t_r_lon).^2.*sum(param.m);
    control.K.I = 0.25;
    control.K.D = 2.*param.zeta_lon.*2.2./param.t_r_lon.*sum(param.m);
    control.x_names = ["h";"h_{dot}"];
    control.u_names = ["F"];
    control.impose_sat = true;
    core.functions.controllers(1) = controllers(control,core);
    
    % \z_{v}
    control.K.P = (2.2./param.t_r_lat_z).^2.*(param.Jc + 2.*param.m(1).*param.d.^2);
    control.K.I = 0.1;
    control.K.D = 2.*param.zeta_lat_z.*2.2./param.t_r_lat_z.*(param.Jc + 2.*param.m(1).*param.d.^2);
    control.x_names = ["z_{v}";"z_{v}_{dot}"];
    control.u_names = ["\tou"];
    control.impose_sat = false;
    core.functions.controllers(2) = controllers(control,core);
    
    % \theta
    control.K.P = 2.2.^2./(-param.g.*param.t_r_lat_theta.^2);
    control.K.I = 0.001;
    control.K.D = (param.mu./sum(param.m) - 2.*param.zeta_lat_theta.*2.2./param.t_r_lat_theta)...
                        ./(param.g);
    control.x_names = ["\theta";"\theta_{dot}"];
    control.u_names = ["\tou"];
    control.impose_sat = false;
    core.functions.controllers(2).cascade = controllers(control,core);
elseif strcmp(control.controller_type,controllers.SS)
    % h Position
    control.K = gains_SS(t_r_lon,zeta_lon,p_i_lon,param.A([1,4],[1,4]),param.B([1,4],[1]),param.C_r([1],[1,4]));
    control.x_names = ["h";"h_{dot}"];
    control.u_names = ["F"];
    control.impose_sat = true;
    core.functions.controllers(1) = controllers(control,core);
    
    % z_{v} Position
    control.K = gains_SS(t_r_lat,zeta_lat,p_i_lat,param.A([2,3,5,6],[2,3,5,6]),param.B([2,3,5,6],[2]),param.C_r([2],[2,3,5,6]));
    control.x_names = ["z_{v}";"\theta";"z_{v}_{dot}";"\theta_{dot}"];
    control.u_names = ["\tou"];
    control.impose_sat = false;
    core.functions.controllers(2) = controllers(control,core);
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