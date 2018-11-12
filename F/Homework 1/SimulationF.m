
HW1ParamF;

animation = animateF(param,sim);
display = plot_data(["z_t - Position of Target (m)",...
    "z_v - Position of VTOL (m)","h - Height of VTOL (m)",...
    "theta - Orientation of VTOL (Degrees)"],...
    [param.initial_z_t,param.initial_z_v,param.initial_h,param.initial_theta]);

[input,time] = function_generator(sim);
theta = input.*45;
position = -input.*(sim.width-1)./2;
vtol = [input.*2,(input + 3)];

printwatch = tic;
stopwatch = tic;



for i = 1:length(input)
    while toc(stopwatch) < time(i)
    end
    if toc(printwatch) > sim.publish
        printwatch = tic;
        animation.redraw(vtol(i,:),theta(i),position(i));
        display.add_data(["z_t - Position of Target (m)",...
        "z_v - Position of VTOL (m)","h - Height of VTOL (m)",...
        "theta - Orientation of VTOL (Degrees)"],...
        [position(i),vtol(i,1),vtol(i,2),theta(i)],time(i));
        if ((vtol(i,2)-cos(theta(i))*(param.propX+param.d)) <= 0)
            disp("BANG! Oh no! The VTOL crashed!")
            break
        elseif (vtol(i,2)>sim.height) || (abs(vtol(i,1))>sim.width./2)
            disp("Oh no! The VTOL left the screen")
            break
        end
    end
end
toc(stopwatch)
