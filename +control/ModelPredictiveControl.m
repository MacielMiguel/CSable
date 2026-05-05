classdef ModelPredictiveControl < core.AbstractController
    % MODELPREDICTIVECONTROL MPC implementation for the quadruped
    
    properties
        Horizon
        Q
        R
    end
    
    methods
        function obj = ModelPredictiveControl(Ts, horizon)
            obj@core.AbstractController(Ts);
            obj.Horizon = horizon;
            obj.Q = eye(4);
            obj.R = eye(2);
        end
        
        function action = computeAction(obj, state, ref)
            % TODO: Implement MPC optimization here
            action = [0; 0];
        end
    end
end
