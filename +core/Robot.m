classdef Robot < handle
    % ROBOT Class representing the quadruped robot
    
    properties
        Params % Robot parameters
        State  % Current state (positions, velocities)
    end
    
    methods
        function obj = Robot(params)
            obj.Params = params;
            obj.State = struct('q', [], 'dq', []);
        end
        
        function updateState(obj, q, dq)
            obj.State.q = q;
            obj.State.dq = dq;
        end
    end
end
