classdef animate < handle
    
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
            
            % Unpack data
            % Display Window Info
            self.window = sim.window;
            self.publish = sim.publish;
            % Playback Info
            self.real_time = sim.real_time;
            self.start = sim.start;
            self.step = sim.step;
            self.names  = sim.names;
            self.hard_stop = param.hard_stop;
            self.animation = sim.animation;
            % Initial Conditions
            self.x_0 = param.x_0;
            self.u_0 = param.u_0;
            self.r_0 = param.r_0;
            % Handle to the function that draws the simulation
            self.get_drawing = param.get_drawing;
            % General Parameters for passing information
            self.param = param;
            self.sim = sim;
            
            % Assign figure numbers
            if self.animation
                self.animation_figure = 1;
                self.plot_figure = 2;
            else
                self.plot_figure = 1;
            end
            
            % Animation Figure
            if self.animation
                
                % Get points
                drawing = self.get_drawing(self.x_0,self.param);

                % Extract Data
                self.points = drawing{1};
                self.color = drawing{2};
                
                % Plot
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
            
            % Plot Data Figure
            figure(self.plot_figure), clf
            % States and commanded positions
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
            
            % Input
            for j = 1:length(self.u_0)
                
                subplot(length(self.names),1,j+i,'FontSize', 10)
                hold on
                self.u_plots(j) = plot(sim.start,self.u_0(j),'linewidth',1.5);
                grid on
                xlabel('t - Time (s)')
                ylabel(self.names(i+j))
                hold off
                
            end
            
            % Reactivate the animation figure
            figure(1)
        end
        
        function redraw(self,x)
            
            % Place the points of the drawing in the new location
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
            
            % Get the time vector and append the new time.
            t = [get(self.x_plots(1),'XData'),t];
            
            % For each variable to be updated
            for i = 1:(length(self.names) - length(self.u_0))
                
                % Get the old state data and append the new data
                data_to_disp = [get(self.x_plots(i),'YData'),x(i,:)];
                % Update the chart data
                set(self.x_plots(i),'XData',t,'YData',data_to_disp);
                
                % Get the old command data and append the new data
                data_to_disp = [get(self.r_plots(i),'YData'),r(i,1:end)]; % -1 for short
                % Update the chart data
                set(self.r_plots(i),'XData',t,'YData',data_to_disp);
                
            end
            
           for i = 1:length(u(:,1))
               
                % Get the old input and append the new data
                data_to_disp = [get(self.u_plots(i),'YData'),u(i,:)];
                % Update the chart data
                set(self.u_plots(i),'XData',t,'YData',data_to_disp);
                
            end
            
            % Redraw
            drawnow
        end
        
        function play(self,x,u,r,t)
            
            if self.animation
                
                % Start timers for accurate timing
                printwatch = tic; % Timer that helps skip unessisary images
                stopwatch = tic; % Timer that iterates through each image

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

                        % Update the dispaly
                        self.redraw(x(:,i));
                        self.add_data(x(:,i),u(:,i),r(:,i),t(i))
                        
                        % End the simulation if physical limits are
                        % exceeded
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
                % Update the plots only
                self.add_data(x,u,r,t(2:end))
            end
        end
    end
end

