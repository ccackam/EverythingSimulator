classdef dynamics < handle
    %DYANMICS class handles the dynamics of the system. values stored in
    %this class include:
    %        
    %     z
    %     z_dot
    %     z_ddot
    %     F
    %     b
    %     k
    %     m
    %     time
    %     step
    %     stop
    %
    % Methods include
    %
    %       self = dynamics(param,sim)
    %       x_dot = eqs_motion(self,t,state)
    %       [position,current_time] = propagate(self)
    %       [position,time_vector] = simulate(self,input)
    
    properties
        x
        u
        param
        theoretical_param
        time
        step
        stop
        eqs_motion
        control_architecture
        controller
        D_in_param
        D_in_u
        D_out
        N
        uncertian_param
        uncertian_u
        uncertian_x
        uncertian_N
        implement_uncertainty
    end
    
    methods
        function self = dynamics(param,sim)
            %DYNAMICS This function initializes the object.
            % Inputs include: param & sim - both are objects with the
            % folowing values:
            % param: z,z_dot,d_ddot,F,b,k,m
            % sim: start,step,end
            self.x = param.x_0;
            self.u = param.u_0;
            self.theoretical_param = param;
            self.eqs_motion = param.eqs_motion;
            self.control_architecture = param.control_architecture;
            self.controller = param.controller;
            self.time = sim.start;
            self.step = sim.step;
            self.stop = sim.end;
            self.uncertian_param = param.uncertian_param;
            self.uncertian_u = param.uncertian_u;
            self.uncertian_x = param.uncertian_x;
            self.uncertian_N = param.uncertain_N;
            self.D_in_param = param.D_in_param;
            self.D_in_u = param.D_in_u;
            self.D_out = param.D_out;
            self.N = param.N;
            self.implement_uncertainty = param.implement_uncertainty;
            
            if self.implement_uncertainty
                % set biases in uncertianties
                self.D_in_param.bias = self.D_in_param.bias.*(rand(1,length(self.D_in_param.bias)).*2-1);
                self.D_in_u.bias = self.D_in_u.bias.*(rand(1,length(self.D_in_u.bias)).*2-1);
                self.D_out.bias = self.D_out.bias.*(rand(1,length(self.D_out.bias)).*2-1);
                self.N.bias = self.N.bias.*(rand(1,length(self.N.bias)).*2-1);

                % set param uncertianty
                self.param = self.uncertainty(self.theoretical_param,self.D_in_param,param.uncertian_param);
            else
                self.param = param;
            end
        end
        
        function results = propagate(self)
            %PROPAGATE Using ODE45 this method propigates the dynamics of 
            %the system. No inputs are needed but time and position at the
            %next time step is returned.
            
            % Update Time
            self.time = self.time + self.step;
            % t_span = [0,self.step];
            
            
            k1 = self.eqs_motion(self.time,self.x,self.u,self.param);
            k2 = self.eqs_motion(self.time,self.x + self.step/2*k1, self.u,self.param);
            k3 = self.eqs_motion(self.time,self.x + self.step/2*k2, self.u,self.param);
            k4 = self.eqs_motion(self.time,self.x + self.step*k3, self.u,self.param);
            new_state = self.x + self.step/6 * (k1 + 2*k2 + 2*k3 + k4);
            
            % ODE45
            % [t_out,new_state] = ode45(@(t,x) self.eqs_motion(t,x,self.u,self.param),t_span,self.x);
            
            
            % Impliment uncertainty
%             if self.implement_uncertainty
%                 results = self.uncertainty(new_state(end,:).',self.D_out,self.uncertian_x);
%             else
%                 results = new_state(end,:).';
%             end
            
            % Impliment uncertainty
            if self.implement_uncertainty
                results = self.uncertainty(new_state,self.D_out,self.uncertian_x);
            else
                results = new_state;
            end
                
            % Unpack results
            self.x = results;
        end
        
        function [x_history,u_history,r_history] = simulate(self,r)
            %SIMULATE This function takes the input force and outputs a
            %position vector and a time vector. This function will need
            %to be altered to impliment a control algorithem.
            
            % Initialize
            x_history = zeros(length(self.x),floor(self.stop./self.step));
            u_history = zeros(length(self.u),length(x_history));
            % Note the r_history can not be prealocated for speed because
            % we are unsure of it's size. That depends on the number of
            % control cascades applied to it.
            
            % Iterate through each timestep
            for i = 1:length(x_history)
                
                % Impliment uncertianty
                if self.implement_uncertainty
                    measurements = self.uncertainty(self.x,self.N,self.uncertian_N);
                else
                    measurements = self.x;
                end
                    
                % Set current force *Impliment Control Algorithem Here*
                output = self.control_architecture(self.controller,measurements,r(:,i),self.theoretical_param);
                
                % Unpack
                self.u = output{1};
                
                % Save r history
                r_history(:,i) = output{2};
                
                % Save Input Vector
                u_history(:,i) = self.u;
                
                % Propagate
                x_history(:,i) = self.propagate();
            end
        end
        
        function output = uncertainty(self,input,uncertainty,keys)    
            
            scale = uncertainty.random.*(rand(1,length(uncertainty.random)).*2-1);
            
            if isstruct(input)
                output = input;
                for i = 1:length(keys)
                    variable = keys{i};
                    code = ['output.',variable,' = input.',variable,'+ scale(i) + uncertainty.bias(i);'];
                    eval(code)
%                     disp(variable)
%                     eval(['disp(input.',variable,')'])
%                     eval(['disp(output.',variable,')'])
                end
            else
                output = zeros(size(input));
                for i = 1:length(input)
                    if keys(i)
                        output(i) = input(i) + scale(i) + uncertainty.bias(i);
                    else
                        output(i) = input(i);
                    end
                end
            end
            
        end
        
    end
end

