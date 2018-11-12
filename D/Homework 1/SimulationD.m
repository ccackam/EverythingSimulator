HW1ParamD;

animation = animateD(param,sim);
display = plot_data("x - Position (m)",0);

[input,time] = function_generator(sim);

printwatch = tic;
stopwatch = tic;

video = [];

for i = 1:length(input)
    while toc(stopwatch) < time(i)
    end
    if toc(printwatch) > sim.publish
        printwatch = tic;
        pause(sim.step)
        animation.redraw(input(i));
        display.add_data("x - Position (m)",input(i),time(i))
    end
end
toc(stopwatch)