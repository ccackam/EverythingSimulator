% For Formating
r = r.';
t = t.';

% Create Objects

animation = animate(param,sim);    % This is the animation that handles  
                                    %   the display of results
system = dynamics(param,sim);       % This is the dynamic model of the 
                                    %   system it simulates the system.
                                        
% Model System
[x,u,r] = system.simulate(r);   % This tells the system object
                                            %   to model the system's  
                                            %   responce with the given 
                                            %   input

% Display Animation                                            
animation.play(x,u,r,t)               % This takes the output of the 
                                            %   system and displays it as 
                                            %   an animation.
                                            