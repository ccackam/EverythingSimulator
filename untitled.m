% full state feedback for topic 13
clear all
clc
% reference input issues

%

t_r = 0.14;
zeta = 1;

w_n = 2.2/t_r;

num = [1];
den = [1,2.*zeta.*w_n,w_n.^2];

h = tf(num,den);
step(h)


% param.l = 0.5;      % m
% param.m = [0.35,2]; % kg - 1=ball 2=beam
% param.g = 9.81;      % m/s^2
% 
% % System Limits
% param.sat_lim.high = 15;
% param.sat_lim.low = -15;
% param.hard_stop.high(1,1) = 0.5; 
% param.hard_stop.high(2,1) = pi./2;
% param.hard_stop.low(1,1) = 0.0;
% param.hard_stop.low(2,1) = -pi./2;
% 
% % State Space Equations
% l = param.l;
% m = param.m;
% g = param.g;
% J = (1/3*l^2*m(2)+m(1)*(l/2)^2);
% a = [0,   0,  1,  0;
%            0,   0,  0,  1;
%            0,  -g,  0,  0;
%            g*m(1)/J,  0,  0,  0];
% b = [0;0;0;l./J];
% c_r = [1,0,0,0];
% c = [1,0,0,0;
%              0,1,0,0];
% d = [0;0];
% 
% t_r_z = 0.14;
% t_r_theta = t_r_z.*8;
% t_r = [t_r_z,t_r_theta];
% zeta_z = 0.707;
% zeta_theta = 0.707;
% zeta = [zeta_z,zeta_theta];
% 
% [k,Nbar] = gains_SS(t_r,zeta,a,b,c_r);
% 
% % a=[1 1;1 2];b=[1 0]';c=[1 0];d=0;
% % k=[14 57];
% % Nbar=-15; 
% 
% sys1=ss(a-b*k,b,c,d);
% sys2=ss(a-b*k,b*Nbar,c,d);
% opt = stepDataOptions('InputOffset',0.25,'StepAmplitude',0.4);
% %t=[0:.025:4];
% [y,t,x]=step(sys1,opt);
% [y2,t2,x2]=step(sys2,opt);
% 
% plot(t,y,'--',t2,y2,'LineWidth',2);
% axis([0 4 -1 1.2]);
% grid;
% legend('u=r?Kx','u=Nbar r?Kx','Location','SouthEast')
% xlabel('time (sec)');ylabel('Y output');title('Step Response') 