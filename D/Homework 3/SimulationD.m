% Load Data
HW3ParamD;

% Create Objects
animation = animateD(param,sim);    % This is the animation that handles  
                                    %   the display of results
system = dynamicsD(param,sim);       % This is the dynamic model of the 
                                    %   system it simulates the system.

% Create Input
[input,~] = function_generator(sim);    % Current input is a step system or
                                        %   or function generator.

% Model System
[position,time] = system.simulate(input);   % This tells the system object
                                            %   to model the system's  
                                            %   responce with the given 
                                            %   input

% Display Animation                                            
animation.play(position,time)               % This takes the output of the 
                                            %   system and displays it as 
                                            %   an animation.