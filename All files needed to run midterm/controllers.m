classdef controllers < handle
    
    properties
        controller_type
        param
        K
        u
        add_equilibrium
        e
        x
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
    end
    properties (Constant)
        PID = 'PID';
        SS = 'SS';
    end
    
    methods
        function self = controllers(param,sim)
            self.controller_type = param.controller_type;
            self.K = param.K;
            self.param = param;
            self.add_equilibrium = param.add_equilibrium;
            self.u.e = param.u.e;
            self.dt = sim.step;
            self.u.I = 0;
            self.u.D = 0;
            self.index = logical(param.index);
            x_0 = param.x_0(self.index);
            self.e = param.command_0-x_0(1);
            self.x = x_0;
            self.beta = (2.*param.sigma - self.dt)./(2.*param.sigma + self.dt);
            self.derivative_source = param.derivative_source;
            self.anti_windup = param.anti_windup;
            self.windup_limit = param.windup_limit;
            self.impose_sat = param.impose_sat;
            self.sat_lim.high = param.sat_lim.high;
            self.sat_lim.low = param.sat_lim.low;
            self.count = 0;
            self.x_e = param.x_e(self.index);

        end
         
        function u = master(self,state,commanded)
            switch self.controller_type
                case self.PID
                    u = self.execute_PID(state,commanded);
                case self.SS
                    u = self.execute_SS(state,commanded);
                otherwise
                    error('Invalid Controller','Can not identify a controller by that name')
            end
        end
        
        function u = execute_PID(self,x_complete,r)
            
            x = x_complete(self.index);
            
            e_new = (r - x(1));
            x_new = x;
            
            if strcmp(self.derivative_source,'measure')
                self.u.D = -x(2);
            elseif strcmp(self.derivative_source,'error')
                self.u.D = self.dirty_direvative(self.u.D,e_new,self.e);
            else
                self.u.D = self.dirty_direvative(self.u.D,self.x(1),x_new(1));
            end
            self.u.P = e_new;
            if (strcmp(self.anti_windup,'derivative') || strcmp(self.anti_windup,'both')) && (self.u.D > self.windup_limit)
                self.u.I = self.u.I;
            else
                self.u.I = self.u.I + self.dt./2.*(self.e + e_new);
            end
            
            u.tilda = self.K.D.*self.u.D + self.K.P.*self.u.P + self.K.I.*self.u.I;
            
            if self.add_equilibrium
                u = u.tilda + self.u.e(x_complete,self.param);
            else
                u = u.tilda;
            end
            
            self.e = e_new;
            self.x = x_new;
            
            if self.impose_sat
                u_sat = self.saturate(u);
                if (strcmp(self.anti_windup,'saturation') || strcmp(self.anti_windup,'both')) && self.K.I
                    self.u.I = self.u.I - 1./self.K.I.*(u - u_sat);
                end
                u = u_sat;
            end
        end
        
        function u = execute_SS(self,x_complete,r)
            
            x = x_complete(self.index);
            
            % Investigate changing this to C_r*x_complete
            x_new = x;
            e_new = (r - x(1));

            if strcmp(self.derivative_source,'position')
                x((length(x)./2+1):end) = self.dirty_direvative(x((length(x)./2+1):end),self.x(1:(length(x)./2)),x_new(1:(length(x)./2)));
            end
            
            if self.count == 500
                a = 1;
            else
                self.count = self.count + 1;
            end
            if isfield(self.K,'k_r')
                u.tilda = -self.K.K*(x-self.x_e) + self.K.k_r*(r-self.x_e(1));
            elseif isfield(self.K,'k_i')
                
                if (strcmp(self.anti_windup,'derivative') || strcmp(self.anti_windup,'both')) && (self.u.D > self.windup_limit)
                    self.u.I = self.u.I;
                else
                    self.u.I = self.u.I + self.dt./2.*(self.e + e_new);
                end
                
                u.tilda = -self.K.K*(x-self.x_e) - self.K.k_i*(self.u.I);
            end

            if self.add_equilibrium
                u = u.tilda + self.u.e(self.param.x_e,self.param);
            else
                u = u.tilda;
            end

            self.x = x_new;
            self.e = e_new;
        end
        
        function execute_PBJ(self,x,r)
            e = (r - x(self.index));
            v = -x(self.index+length(x)./2);
            
            syms x(t)
            ode = self.param.m*diff(x,t,2)-self.param.b.*diff(x,t)-self.param.k.*x == self.sat_lim.low;
            cond = [x(0)==x(1),diff(x(0))==x(2)]
            
            x_of_t = dsolve(ode,cond)
            
            if v < self.K.lim
            end
            
            
            
            end
            
        function x_dot = dirty_direvative(self,old_x_dot,x,old_x)
            x_dot = self.beta.*old_x_dot + (1-self.beta)./self.dt.*(x-old_x);
        end
        
        function F = saturate(self,F)
            for i = 1:length(F)
                if F(i) >= self.sat_lim.high
                    F = self.sat_lim.high;
                    warning('SATURATING: High saturation limit reached.')
                elseif F <= self.sat_lim.low
                    F = self.sat_lim.low;
                    warning('SATURATING: Low saturation limit reached.')
                end
            end
        end
    end
end

