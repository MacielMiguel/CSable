% MAIN_HARDWARE Entry point to run the controller on the real robot

% 1. Load configuration and basic initializations
addpath('config');
params = robot_params();

% 2. Initialize the interface with the Dynamixel (X-Series XM430-W350)
dxl = hardware.DynamixelInterface(params.hw);
dxl.init(); % Open the port and enable torque

% 3. Initialize the Open Loop Controller
% Pass the sampling time (0.01s) and the kinematics parameters
Ts = 0.01;
ctrl = control.OpenLoopControl(Ts, params);

% 4. Define our simple trajectory (From Point A to Point B)
pointA = [0.0; -0.20];  % [X, Z] Initial position (in meters)
pointB = [0.05; -0.20]; % [X, Z] Final position
total_time = 2.0;       % Seconds to complete the movement
steps = total_time / Ts;

disp('Starting leg movement...');

try
    % Control loop: runs every Ts seconds
    for k = 1:steps
        % A. PLANNING: Simple linear interpolation
        % Find out where the robot should be *at this exact instant k*
        percentage = k / steps;
        current_ref = pointA + (pointB - pointA) * percentage;
        
        % B. CONTROL CALCULATION: Ask the controller for the angles
        % (Since it's open loop, we pass an empty state [], relying on Inverse Kinematics)
        target_angles = ctrl.computeAction([], current_ref);
        
        % C. CONVERSION: Radians -> Motor "Steps" (0 to 4095)
        dxl_pos_motor1 = utils.rad2dxl(target_angles(1));
        dxl_pos_motor2 = utils.rad2dxl(target_angles(2));
        
        % D. ACTUATION: Send the target positions to the physical motors
        dxl.writePosition(params.hw.DXL_IDS(1), dxl_pos_motor1);
        dxl.writePosition(params.hw.DXL_IDS(2), dxl_pos_motor2);
        
        % Pause to maintain the correct sampling frequency
        pause(Ts);
    end
catch ME
    disp('Error during movement. Shutting down for safety...');
    dxl.cleanup();
    rethrow(ME);
end

% 5. Disable the motors at the end
dxl.cleanup();
disp('Movement completed successfully.');
