function [points,time] = function_generator(sim)

time = [sim.start:sim.step:sim.end].';

switch sim.input
    case 'sine'
        points = sim.amplitude.*sin(time.*(2.*pi)./sim.period + 2.*pi.*sim.phase_delay) + sim.offset;
    case 'saw'
        points = sim.amplitude.*sawtooth(time.*(2.*pi)./sim.period  + 2.*pi.*sim.phase_delay) + sim.offset;
    case 'square'
        points = sim.amplitude.*square(time.*(2.*pi)./sim.period  + 2.*pi.*sim.phase_delay) + sim.offset;
    otherwise
        points = zeros(length(time),1)+sim.offset;
        
end

% Plot for visulization
% figure(3), clf;
% hold on
% plot(time,points);
% hold off

end

