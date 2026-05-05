classdef OpenLoopControl < core.AbstractController
    % OPENLOOPCONTROL Open Loop control implementation
    
    properties
        Params % Parameters to use during the execution (e.g., kinematics)
    end
    
    methods
        function obj = OpenLoopControl(Ts, params)
            obj@core.AbstractController(Ts);
            obj.Params = params;
        end
        
        function action = computeAction(obj, state, ref)
            % ref: [X; Z] or [X; Y; Z] target position of the foot in space
            % state: current state (positions and velocities)
            
            % 1. Use inverse kinematics to transform the desired position (ref)
            %    into target joint angles (radians).
            target_angles_rad = kinematics.inverse_kinematics(ref, obj.Params);
            
            % The output action of this controller is the target joint angles
            action = target_angles_rad;
        end
    end
end
%dfgtyuikjnbgvfd