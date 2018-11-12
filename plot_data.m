classdef plot_data < handle
    %Plots given data
    
    properties
        names
        plots
    end
    
    methods
        function self = plot_data(names,initial)
            %Constructor
            self.names  = names;
            
            figure(2), clf
            hold on
            for i = 1:length(names)
                subplot(length(names),1,i,'FontSize', 15)
                self.plots(i) = plot(0,initial(i),'linewidth',1.5);
                grid on
                xlabel('t - Time (s)')
                ylabel(names(i))
            end
            hold off
        end
        
        function add_data(self,names,data,time)
            %Adds to charts
            time = [get(self.plots(1),'XData');time];
            for i = 1:length(names)
                index = strcmp(names(i),self.names);
                data_to_disp = [get(self.plots(index),'YData'),data(index)];
                set(self.plots(index),'XData',time,'YData',data_to_disp);
            end
            drawnow
        end
    end
end

