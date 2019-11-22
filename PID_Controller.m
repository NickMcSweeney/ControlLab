classdef PID_Controller

    properties (Access = private)
        old_e
        set_value
        E
        dt
        kp
        ki
        kd
    end

    methods
        function obj = PID_Controller(set, p, i, d, deltat)
            obj.set_value = set;
            obj.kp = p;
            obj.ki = i;
            obj.kd = d;
            obj.dt = deltat;
            obj.old_e = 0;
            obj.E = 0;
        end
        function setK(obj, p, i, d)
            obj.kp = p;
            obj.ki = i;
            obj.kd = d;
        end
        function u = pid(obj, measure)
            e = obj.set_value - measure;
            de = (e - obj.old_e)/obj.dt;
            obj.E = e*obj.dt + obj.E;

            u = obj.kp * e + obj.ki * obj.E + obj.kd * de;
            obj.old_e = e;
        end
    end
end
