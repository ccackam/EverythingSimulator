%% Header
% This is the master function which handles the simulation

% For Formating
r = r.';
t = t.';

%% Create Objects
% Object for display of results
animation = animate(param,sim);  

% Object simulating the system
system = dynamics(param,sim);
   
%% Simulate the system
[x,u] = system.simulate(r); 

%% Display results                                            
animation.play(x,u,r,t)
                                            