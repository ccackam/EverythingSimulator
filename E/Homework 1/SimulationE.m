
HW1ParamE;

animation = animateE(param,sim);
display = plot_data(["Theta - Angle (°)","r - Position Along Beam (m)"],...
    [param.initial_theta,param.initial_r]);

[input,time] = function_generator(sim);
theta = input.*45;
position = (input+1)./2.*param.l;

printwatch = tic;
stopwatch = tic;

for i = 1:length(input)
    while toc(stopwatch) < time(i)
    end
    if toc(printwatch) > sim.publish
        printwatch = tic;    
        pause(sim.step);
        animation.redraw(theta(i),position(i));
        display.add_data(["Theta - Angle (°)","r - Position Along Beam (m)"],...
            [theta(i),position(i)],time(i))
        if (position(i) > param.l) || (position(i) < 0) ||...
                (abs(theta(i)) > 90)
            disp("Oh no! The ball fell off the beam!")
            break
        end
    end
end
toc(stopwatch)