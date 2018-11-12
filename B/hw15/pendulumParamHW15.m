% inverted Pendulum parameter file
addpath ./.. % adds the parent directory to the path
pendulumParam % general pendulum parameters

% Compute inner and outer open-loop transfer functions
P_in = tf([-2/P.m2/P.ell],[1,0,-2*(P.m1+P.m2)*P.g/P.m2/P.ell]);
P_out = tf([P.g],[1,0,0]);

% display bode plots of transfer functions
figure(1), clf, bode(P_in), grid on
figure(2), clf, bode(P_out), grid on





