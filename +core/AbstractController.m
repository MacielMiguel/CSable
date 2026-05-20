classdef (Abstract) AbstractController < handle
    % ABSTRACTCONTROLLER Base class for all robot controllers
    
    properties
        Ts % Sample time
    end
    
    methods
        function obj = AbstractController(Ts)
            if nargin > 0
                obj.Ts = Ts;
            else
                obj.Ts = 0.01;
            end
        end
    end
    
    methods (Abstract)
        % Compute the control action
        % Inputs:
        %   state - current state of the robot
        %   ref   - reference trajectory or target
        % Outputs:
        %   action - control inputs (e.g., torques or positions)
        action = computeAction(obj, state, ref)
    end
end
