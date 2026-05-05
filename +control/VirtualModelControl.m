classdef VirtualModelControl < core.AbstractController
    % VIRTUALMODELCONTROL Compliance controller using virtual springs/dampers
    
    properties
        Params    % Robot kinematics and dynamics parameters
        Kp        % Proportional gain matrix (Virtual Spring stiffness)
        Kd        % Derivative gain matrix (Virtual Damper friction)
        MaxTorque % Safety limit for output torques [Nm]
    end
    
    methods
        function obj = VirtualModelControl(Ts, params)
            obj@core.AbstractController(Ts);
            obj.Params = params;
            
            % TUNE THESE VALUES! 
            % Kp defines how "stiff" the virtual spring is (N/m).
            % Kd defines how much it resists fast movements (Ns/m).
            % We want X to be stiff for pushing, and Z to be slightly softer for sand.
            obj.Kp = [500,   0; 
                        0, 300]; 
                        
            obj.Kd = [10,  0; 
                       0, 10];   
            
            % SAFETY LIMITS: 
            % The Dynamixel XM430-W350 stall torque is roughly 4.1 Nm at 12V.
            % We clamp it at 3.0 Nm to prevent overheating and stripping the gears.
            obj.MaxTorque = [3.0; 3.0]; 
        end
        
        function action = computeAction(obj, state, ref)
            % ref: [X_ref; Z_ref] desired foot position
            % state.pos: [X_act; Z_act] current physical foot position
            % state.vel: [VX_act; VZ_act] current physical foot velocity
            % state.angles: [theta1; theta2] current joint angles
            
            % 1. Calculate Cartesian Error (Position and Velocity)
            pos_error = ref - state.pos;
            
            % Assuming the reference target is static for that specific millisecond (velocity = 0).
            % If your main loop provides reference velocities, you'd use (ref_vel - state.vel)
            vel_error = [0; 0] - state.vel; 
            
            % 2. Calculate Virtual Force: F = Kp*e + Kd*e_dot
            F_virtual = (obj.Kp * pos_error) + (obj.Kd * vel_error);
            
            % 3. Get the Jacobian Matrix
            % Your friend's kinematics module needs to supply the 2x2 Jacobian 
            % evaluated at the CURRENT joint angles.
            J = kinematics.compute_jacobian(state.angles, obj.Params);
            
            % 4. Map Virtual Force to Joint Torques: tau = J^T * F
            tau = J' * F_virtual;
            
            % 5. SAFETY CHECK: Torque Clamping
            % Prevent asking the motors for more torque than they can safely deliver.
            tau = max(min(tau, obj.MaxTorque), -obj.MaxTorque);
            
            % The output action is the target Torques [Nm] for the two motors
            action = tau;
        end
    end
end