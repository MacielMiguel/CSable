classdef OpenLoopControl < core.AbstractController
    % OPENLOOPCONTROL Open Loop control implementation
    
    properties
        Params         % Parameters to use during the execution (e.g., kinematics)
        PreviousAction % Memory of the last valid commanded angles [rad]
        MaxDeltaAngle  % Maximum allowed angle change per time step (velocity limit)
    end
    
    methods
        function obj = OpenLoopControl(Ts, params)
            obj@core.AbstractController(Ts);
            obj.Params = params;
            
            % Initialize with an array of zeros (assuming 2 or 3 DOF depending on your leg)
            % Update this dimension based on how many motors are in one leg.
            obj.PreviousAction = [0; 0]; 
            
            % SAFETY: Set a hard speed limit. 
            % e.g., Allow a max of 60 degrees per second.
            max_speed_rad_per_sec = 60 * (pi / 180); 
            obj.MaxDeltaAngle = max_speed_rad_per_sec * Ts; 
        end
        
       function action = computeAction(obj, ~, ref)
            % ref: [X; Z] or [X; Y; Z] target position of the foot in space
            % state: current state (positions and velocities)
            
            % 1. Calculate Target (IK solver)
            target_angles_rad = kinematics.inverse_kinematics(ref, obj.Params);
            
            % 2. SAFETY CHECK: Singularity / Out-of-Bounds
            % If IK fails to find a solution, it often returns NaN or complex numbers.
            if any(isnan(target_angles_rad)) || ~isreal(target_angles_rad)
                warning('OpenLoopControl: IK Solution Invalid (Singularity/Out of Reach). Holding previous position.');
                action = obj.PreviousAction;
                return; % Exit early, do not send bad data
            end
            
            % 3. SAFETY CHECK: Velocity Clamping (Rate Limiting)
            % Calculate how far the motor wants to move this step.
            delta_angle = target_angles_rad - obj.PreviousAction;
            
            % Clamp the movement so it cannot exceed our safety limit.
            delta_angle = max(min(delta_angle, obj.MaxDeltaAngle), -obj.MaxDeltaAngle);
            
            % Calculate the final, safe commanded angle
            action = obj.PreviousAction + delta_angle;
            
            % 4. Update memory for the next control loop
            obj.PreviousAction = action;
        end
    end
end
