clear all
clc

HW3ParamE;

system = dynamicsE(param,sim);

system.F = 0;

[time,out] = ode45(@system.eqs_motion,[0,1000],[0,param.l./2,0,0]);

figure(1), clf
hold on
plot(time,out(:,1))
%plot(time,out(:,2))
hold off