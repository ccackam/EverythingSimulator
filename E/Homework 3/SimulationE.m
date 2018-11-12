% Load Data
HW3ParamE;

% Create Objects
animation = animateE(param,sim);    % This is the animation that handles 
                                    %   the display of results
system = dynamicsE(param,sim);       % This is the dynamic model of the
                                    %   system it simulates the system.
                                    
% Create Input
[input,~] = function_generator(sim);    % Current input is a step system or
                                        %   or function generator.
                                        
% Model System
[theta,r,time] = system.simulate(input+sum(param.m)./2.*param.g);   % This tells the system object
                                            %   to model the system's  
                                            %   responce with the given 
                                            %   input                                        

% Display Animation                                            
animation.play(theta,r,time)               % This takes the output of the 
                                            %   system and displays it as 
                                            %   an animation.                                            
                                            