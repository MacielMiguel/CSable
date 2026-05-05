classdef VirtualModelControl < core.AbstractController
    % VIRTUALMODELCONTROL VMC implementation for the quadruped
    
    properties
        Stiffness
        Damping
    end
    
    methods
        function obj = VirtualModelControl(Ts)
            obj@core.AbstractController(Ts);
            obj.Stiffness = 100;
            obj.Damping = 10;
        end
        
        function action = computeAction(obj, state, ref)
            % TODO: Implement VMC logic here
            action = [0; 0];
        end
    end
end
