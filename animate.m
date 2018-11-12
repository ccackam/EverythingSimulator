classdef animate < handle
    %The animate class contains methods and values partaining to the
    %display. Values stored in objects of this type include:
    % 
    %         side
    %         points
    %         height
    %         width
    %         animation_handle
    %         figure_handle
    %         publish
    %         real_time
    %         step
    %         names
    %         plots
    %         initial
    %         start
    %
    % methods include
    % 
    %         self = animate(param,sim)
    %         redraw(self,position)
    %         add_data(self,names,data,time)
    %         play(self,position,time)
    
    properties
        param
        get_drawing
        points
        color
        window
        animation_handle
        figure_handle
        publish
        real_time
        step
        names
        x_plots
        u_plots
        r_plots
        x_0
        u_0
        r_0
        start
        hard_stop
        sim
        animation
        animation_figure
        plot_figure
    end
    
    methods
        function self = animate(param,sim)
            %The animate method constructs an instance of this class. It
            %stores the needed information in object variables and creats
            %a chart to plot the animation. This constructor needs the
            %param and sim objects passed to it.
            
            % Unpack data
            self.window = sim.window;
            self.publish = sim.publish;
            self.real_time = sim.real_time;
            self.start = sim.start;
            self.step = sim.step;
            self.names  = sim.names;
            self.x_0 = param.x_0;
            self.u_0 = param.u_0;
            self.r_0 = param.r_0;
            self.get_drawing = param.get_drawing;
            self.param = param;
            self.sim = sim;
            self.hard_stop = param.hard_stop;
            self.animation = sim.animation;
            
            if self.animation
                self.animation_figure = 1;
                self.plot_figure = 2;
            else
                self.plot_figure = 1;
            end
            
            % Create a figure to animate the output of the system
            if self.animation
                
                % Get points
                drawing = self.get_drawing(param.x_0,self.param);

                % Extract Data
                self.points = drawing{1};
                self.color = drawing{2};
                
                figure(self.animation_figure); 
                clf
                hold on
                for i = 1:length(self.color)
                    self.animation_handle(i) = fill(self.points{i}(:,1),...
                                                 self.points{i}(:,2),...
                                                 self.color{i});
                end
                axis equal
                axis(self.window)
                hold off
            end
            
            % Create a figure to display the needed data for the system.
            figure(self.plot_figure), clf
            for i = 1:(length(self.names) - length(self.u_0))
                subplot(length(self.names),1,i,'FontSize', 10)
                hold on
                self.x_plots(i) = plot(sim.start,self.x_0(i),'linewidth',1.5);
                self.r_plots(i) = plot(sim.start,self.r_0(i),'linewidth',1.5);
                grid on
                xlabel('t - Time (s)')
                ylabel(self.names(i))
                legend("Actual","Commanded")
                hold off
            end
            for j = 1:length(self.u_0)
                subplot(length(self.names),1,j+i,'FontSize', 10)
                hold on
                self.u_plots(j) = plot(sim.start,self.u_0(j),'linewidth',1.5);
                grid on
                xlabel('t - Time (s)')
                ylabel(self.names(i+j))
                hold off
            end
            
            figure(1)
        end
        
        function redraw(self,x)
            %redraw places the box in a new location. This method needs the
            %new z location passed to it.
            
            % Format incoming data.
            drawing = self.get_drawing(x,self.param);
            
            % Extract Data
            self.points = drawing{1};
            self.color = drawing{2};           
            
            % Update the chart data.
            for i = 1:length(self.color)
                set(self.animation_handle(i),...
                    'X',self.points{i}(:,1),...
                    'Y',self.points{i}(:,2))
            end
            
            % Redraw
            drawnow
        end
        
        function add_data(self,x,u,r,t)
            %add_data adds a set of data points to the chart. To opporate a
            %the names of the values to be updated, the data itself, and 
            %the time of the data all need to be passed to the method
            
            % Get the time vector and append the new time.
            t = [get(self.x_plots(1),'XData'),t];
            
            % For each variable to be updated
            for i = 1:(length(self.names) - length(self.u_0))
                % Get the old data and append the new data
                data_to_disp = [get(self.x_plots(i),'YData'),x(i,:)];
                % Update the chart data
                set(self.x_plots(i),'XData',t,'YData',data_to_disp);
                % Get the old data and append the new data
                data_to_disp = [get(self.r_plots(i),'YData'),r(i,:)];
                % Update the chart data
                set(self.r_plots(i),'XData',t,'YData',data_to_disp);
            end
            
           for i = 1:length(u(:,1))
                % Get the old data and append the new data
                data_to_disp = [get(self.u_plots(i),'YData'),u(i,:)];
                % Update the chart data
                set(self.u_plots(i),'XData',t,'YData',data_to_disp);
            end
            
            % Redraw
            drawnow
        end
        
        function play(self,x,u,r,t)
            %play the play method takes the position vector and time vector
            %and iterates through them dispalying them one at a time.
            
            if self.animation
                % Start timers for accurate timing
                printwatch = tic;
                stopwatch = tic;

                % Iterate through each position
                for i = 1:length(x)

                    % Wait for the first timer to indicate it's time to move on
                    % to the next image
                    while (toc(stopwatch) < t(i)) && self.real_time
                    end

                    % If the second timer indicates that it is also time to
                    % display a new image.
                    if (toc(printwatch) > self.publish) || (~self.real_time && ~mod(i,(self.publish./self.step)))

                        % Reset the timer
                        printwatch = tic;

                        % Update teh dispaly
                        self.redraw(x(:,i));
                        self.add_data(x(:,i),u(:,i),r(:,i),t(i))

                        if any(self.hard_stop.high < ...
                                x(1:length(self.hard_stop.high),i)) && ...
                                self.sim.realistic
                            error('HARD STOP: High system limits exceded')
                        elseif any(self.hard_stop.low > ...
                                x(1:length(self.hard_stop.low),i)) && ...
                                self.sim.realistic
                            error('HARD STOP: Low system limits exceded')
                        end
                    end
                end
                % For information's sake, display the time of the output
                toc(stopwatch)
            else
                % Update teh dispaly
                if self.animation
                    self.redraw(x);
                end
                self.add_data(x,u,r,t(2:end))
            end
        end
    end
end

