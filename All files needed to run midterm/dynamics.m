classdef dynamics < handle
    
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
            % Initialize State
            self.x = param.x_0;
            self.u = param.u_0;
            
            % Functions
            self.eqs_motion = param.eqs_motion;
            self.control_architecture = param.control_architecture;
            self.controller = param.controller;
            
            % Simulation Parameters
            self.time = sim.start;
            self.step = sim.step;
            self.stop = sim.end;
            
            % Uncertianty
            self.theoretical_param = param;
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
                % set biases in parameters
                self.D_in_param.bias = self.D_in_param.bias.*(rand(1,length(self.D_in_param.bias)).*2-1);
                self.D_in_u.bias = self.D_in_u.bias.*(rand(1,length(self.D_in_u.bias)).*2-1);
                self.D_out.bias = self.D_out.bias.*(rand(1,length(self.D_out.bias)).*2-1);
                self.N.bias = self.N.bias.*(rand(1,length(self.N.bias)).*2-1);

                % set parameters randomness
                self.param = self.uncertainty(self.theoretical_param,self.D_in_param,param.uncertian_param);
            else
                self.param = param;
            end
            
        end
        
        function results = propagate(self)

            % Update Time
            self.time = self.time + self.step;
            
            % Runga Kuta
            k1 = self.eqs_motion(self.time,self.x,self.u,self.param);
            k2 = self.eqs_motion(self.time,self.x + self.step/2*k1, self.u,self.param);
            k3 = self.eqs_motion(self.time,self.x + self.step/2*k2, self.u,self.param);
            k4 = self.eqs_motion(self.time,self.x + self.step*k3, self.u,self.param);
            new_state = self.x + self.step/6 * (k1 + 2*k2 + 2*k3 + k4);

%             % This is to impliment ode45 this is very slow, but a
%             % possible feature for futher development. 
%             ODE45
%             [t_out,new_state] = ode45(@(t,x) self.eqs_motion(t,x,self.u,self.param),t_span,self.x);
%             
%             
%             Impliment uncertainty
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
        
        function [x_history,u_history] = simulate(self,r)
            
            % Initialize
            x_history = zeros(length(self.x),floor(self.stop./self.step));
            u_history = zeros(length(self.u),length(x_history));
            
            % Iterate through each timestep
            for i = 1:length(x_history)
                
                % Impliment uncertianty
                if self.implement_uncertainty
                    measurements = self.uncertainty(self.x,self.N,self.uncertian_N);
                else
                    measurements = self.x;
                end
                    
                % Implimennt controller
                self.u = self.control_architecture(self.controller,measurements,r(:,i),self.theoretical_param);
                
                % Save history
                u_history(:,i) = self.u;
                
                % Simulate Responce to new input
                x_history(:,i) = self.propagate();
            end
        end
        
        function output = uncertainty(self,input,uncertainty,keys)    
            
            % Scale the uncertianty
            scale = uncertainty.random.*(rand(1,length(uncertainty.random)).*2-1);
            
            % If we're adjusting the parameters
            if isstruct(input)
                
                output = input;
                for i = 1:length(keys)
                    % Only impliment the uncertianty for the variables given.
                    variable = keys{i};
                    code = ['output.',variable,' = input.',variable,'+ scale(i) + uncertainty.bias(i);'];
                    eval(code)
%                     % This is to see the new values if needed
%                     disp(variable)
%                     eval(['disp(input.',variable,')'])
%                     eval(['disp(output.',variable,')'])
                end
                
            else % if we're adjusting any other number
                
                output = zeros(size(input));
                for i = 1:length(input)
                    % Only make uncertian if the key tells us to.
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

