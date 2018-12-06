classdef controllers < handle
    
    properties
        controller_type
        param
        K
        u
        u_e
        add_equilibrium
        e
        dt
        index
        beta
        derivative_source
        anti_windup
        windup_limit
        impose_sat
        sat_lim
        count
        x_e
        r_e
        core
        C_m
        observer
        x_names
        u_names
        r_names
        output_names
        use_names
        error
        cascade
        intigrator_correction
    end
    properties (Constant)
        PID = 'PID';
        SS = 'SS';
    end
    
    methods
        function self = controllers(control,core)
            % Unapck
            self.core = core;
            param = core.param;
            functions = core.functions;
            settings = core.settings;
            
            self.controller_type = control.controller_type;
            self.K = control.K;
            self.use_names = control.x_names;
            self.output_names = control.u_names;
            self.anti_windup = control.anti_windup;
            self.windup_limit = control.windup_limit;
            if isfield(control,'cascade')
                self.cascade = control.cascade;
            end
            
            % General to pass to other functions
            self.core = core;
            self.param = param;
            
            % Functions
            self.u_e = functions.u_e;
            
            % States
            self.x_names = param.x_names;
            self.u_names = param.u_names;
            self.r_names = param.x_names(param.C_r*(1:length(self.x_names)).');
            self.x_e = self.get_compressed_state(self.x_names,self.use_names,param.x_e);
            self.r_e = self.get_compressed_state(self.x_names,self.r_names,param.x_e);
            
            % Settigs
            self.dt = settings.step;
            self.impose_sat = control.impose_sat;
            self.sat_lim.high = param.sat_lim.high;
            self.sat_lim.low = param.sat_lim.low;
            
            % initiallize variables
            self.error = 0;
            self.intigrator_correction = 0;
            self.count = 0;
        end
        
        function u = execute_PID(self,u_P,u_I,u_D)
            u = - self.K.D.*u_D + self.K.P.*u_P + self.K.I.*u_I;
        end
        
        function u = execute_SS(self,x,r,sum_of_error)
            u = -self.K.K*(x-self.x_e) + self.K.k_r*(r-self.r_e) - self.K.I*sum_of_error;
        end
        
        function summed_error = sum_error(self)
            summed_error = self.dt*trapz(self.error);
        end
        
        function saturation_anti_windup(self,u_unsat,u_sat)
            self.intigrator_correction = self.intigrator_correction + 1./self.K.I.*(u_unsat - u_sat);
        end
        
        function derivative_anti_windup(self,velocity)
            if abs(velocity) > self.windup_limit
                self.intigrator_correction = self.intigrator_correction + self.dt*trapz(self.error(end-1:end));
            end
        end
        
        function F = saturate(self,F)
            for i = 1:length(F)
                if F(i) >= self.sat_lim.high
                    F(i) = self.sat_lim.high;
                    warning('SATURATING: High saturation limit reached.')
                elseif F <= self.sat_lim.low
                    F(i) = self.sat_lim.low;
                    warning('SATURATING: Low saturation limit reached.')
                end
            end
        end
        
        function x_out = get_compressed_state(self,options,choices,x_in)
            x_out = zeros(size(choices));
            for i = 1:length(choices)
                if any(strcmp(options,choices(i)))
                    x_out(i) = x_in(strcmp(options,choices(i)));
                end
            end
        end
        
        function u = control(self,full_x,r,d_hat)
            
            % Unpack
            x = self.get_compressed_state(self.x_names,self.use_names,full_x);
            indexes = ~cellfun(@isempty,regexp(self.use_names,".*_{dot}"));
            velocity = x(indexes);
            position = x(~indexes);
            y_r = sum(self.get_compressed_state(self.x_names,self.x_names(self.param.C_r*(1:length(self.x_names)).'),x));
            r = sum(self.get_compressed_state(self.x_names(self.param.C_r*(1:length(self.x_names)).'),self.use_names,r));
            self.error = [self.error,r - y_r];
            
            % Anti-Windup
            if (strcmp(self.anti_windup,'derivative') || strcmp(self.anti_windup,'both'))
                    self.derivative_anti_windup(velocity);
            end
               
            % Execute Controller
            switch self.controller_type
                case self.PID
                    u_P = self.error(end);
                    u_I = self.sum_error() - self.intigrator_correction;
                    u_D = velocity;
                    u = self.execute_PID(u_P,u_I,u_D);
                case self.SS
                    sum_of_error = self.sum_error() - self.intigrator_correction;
                    u = self.execute_SS(x,r,sum_of_error);
            end
            
            if ~isempty(self.cascade)
                u = self.cascade.control(full_x,u);
            else
                % Add Equilibrium
                u_unsat = u + self.get_compressed_state(self.u_names,self.output_names,self.u_e(self.param.x_e,self.param)) - d_hat;
            
                % Anti-Windup
                if self.impose_sat
                    u_sat = self.saturate(u_unsat);
                    if ((strcmp(self.anti_windup,'saturation') || strcmp(self.anti_windup,'both'))) & self.K.I
                        self.saturation_anti_windup(u_unsat,u_sat)
                    end
                    u = u_sat;
                else
                    u = u_unsat;
                end
            end
        end
    end
end

