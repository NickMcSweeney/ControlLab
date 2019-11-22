classdef Integrator
    properties
        deltaT
        integrand
    end
    methods
        function obj = Integrator(fun,dT)
            obj.integrand = fun;
            obj.deltaT = dT;
        end
        % Euler Integrator
        function X_n = euler(obj,x_n,t_n)
            X_n = x_n + obj.deltaT*feval(obj.integrand,x_n,t_n);
        end
        % Fourth order RK scheme
        function X_n = rk4(obj,x_n,t_n)
            k1 = feval(obj.integrand,x_n,t_n);
            k2 = feval(obj.integrand,(x_n+(obj.deltaT/2)*k1),t_n);
            k3 = feval(obj.integrand,(x_n+(obj.deltaT/2)*k2),t_n);
            k4 = feval(obj.integrand,(x_n+obj.deltaT*k3),t_n);
            X_n = x_n + (obj.deltaT/6)*(k1+2*k2+2*k3+k4);
        end
    end
end
