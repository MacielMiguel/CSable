% main_hardware_ellipse.m
% Real-time hardware execution of the OpenLoopControl tracking an ellipse.

clear; clc; close all;

%% 1. Setup Parameters & Dimensions
Ts = 0.01;          % Sample time (100 Hz)
T_end = 4;          % Execution duration (seconds)
t = 0:Ts:T_end;     % Time vector

% Robot Kinematics Parameters
params = struct();
params.dm.L2 = 100.00; % Thigh
params.dm.L3 = 105.73; % Shin

%% 2. Setup Hardware Parameters
% UPDATE THESE SETTINGS TO MATCH YOUR PHYSICAL ROBOT
hw_params = struct();
hw_params.DEVICENAME = 'COM5';          % Windows: 'COM3', Linux: '/dev/ttyUSB0', Mac: '/dev/tty.usb...'
hw_params.BAUDRATE = 1000000;           % Default for X-Series is usually 1000000 or 57600
hw_params.PROTOCOL_VERSION = 2.0;       % X-Series uses Protocol 2.0
hw_params.DXL_IDS = [4, 3];             % [Thigh Motor ID, Crank Motor ID]

% HARDWARE OFFSETS [radians]
% Offset = Physical Motor Angle - Mathematical Model Angle
% From calibration: Motor 'a' (ID 3) has a +143 degree offset.
offset_theta2 = deg2rad(0);   % Update this once you calibrate theta2
offset_a      = deg2rad(-35); % The 35 deg difference you discovered

hw_params.OFFSETS = [offset_theta2, offset_a];

%% 3. Define the Safe 2D Ellipse Trajectory
Xc = 40;     % Shifted forward
Yc = -145;   % Shifted downward
A = 20;      % Ellipse width semi-axis (mm)
B = 10;      % Ellipse height semi-axis (mm)
freq = 0.5;  % 0.5 Hz (1 cycle every 2 seconds)

ref_x = Xc + A * cos(2 * pi * freq * t);
ref_y = Yc + B * sin(2 * pi * freq * t);

%% 4. Initialize Subsystems
% 4a. Initialize Controller & Warm Start Check
controller = control.OpenLoopControl(Ts, params);

init_ref = [ref_x(1); ref_y(1)];
init_q = kinematics.inverse_kinematics(init_ref, params);

if any(isnan(init_q))
    error('WORKSPACE ERROR: The starting point is unreachable. Check your Xc, Yc, A, B.');
end
controller.PreviousAction = init_q;

% 4b. Initialize Hardware
disp('Initializing hardware connection...');
hw_interface = hardware.DynamixelInterface(hw_params);
hw_interface.init();

% Move to the starting position slowly before the fast trajectory begins
disp('Moving to start position. Please stand clear...');
hw_interface.writePosition(hw_params.DXL_IDS, init_q);
pause(2.0); % Give the motors 2 seconds to reach the start point safely

%% 5. Preallocate Logging Arrays
N = length(t);
action_log = zeros(2, N);
actual_pos_log = zeros(2, N); 
ref_pos_log = [ref_x; ref_y];   

disp('Starting trajectory execution...');

%% 6. REAL-TIME HARDWARE CONTROL LOOP
for i = 1:N
    % Mark the exact start time of this loop iteration
    loop_start = tic; 
    
    % 1. Get current reference
    ref = ref_pos_log(:, i);
    
    % 2. Compute Control Action (IK + Limiter)
    action = controller.computeAction([], ref);
    
    % 3. Send to Hardware!
    hw_interface.writePosition(hw_params.DXL_IDS, action);
    
    % 4. Read Actual Hardware Position (for plotting errors)
    % This adds slight delay but is highly valuable for debugging
    act_theta2 = hw_interface.readPosition(hw_params.DXL_IDS(1));
    act_a      = hw_interface.readPosition(hw_params.DXL_IDS(2));
    act_angles = [act_theta2; act_a];
    
    % Calculate the real physical Cartesian location using FK
    act_pos_3d = kinematics.forward_kinematics(act_angles, params);
    actual_pos = act_pos_3d(1:2);
    
    % 5. Log Data
    action_log(:, i) = action;
    actual_pos_log(:, i) = actual_pos;
    
    % 6. Real-time pacing constraint
    % Check how long computation and serial communication took
    elapsed_time = toc(loop_start);
    
    % If the loop finished faster than Ts (0.01s), pause for the remainder
    % so we don't bombard the motors too fast and warp the time scale.
    if elapsed_time < Ts
        pause(Ts - elapsed_time);
    end
end

disp('Trajectory complete. Cleaning up hardware...');

%% 7. Safe Cleanup
% The interface destructor handles torque disable and port closing
hw_interface.cleanup();

%% 8. Plot Final Hardware Tracking Errors
% We plot this after the loop so the graphics rendering doesn't slow down the physical robot
error_x = ref_pos_log(1, :) - actual_pos_log(1, :);
error_y = ref_pos_log(2, :) - actual_pos_log(2, :);

figure('Name', 'Hardware Tracking Errors', 'Color', 'w');
subplot(2,1,1);
plot(t, error_x, 'r', 'LineWidth', 1.5);
grid on; title('Hardware Tracking Error: X-Axis'); ylabel('Error (mm)');

subplot(2,1,2);
plot(t, error_y, 'b', 'LineWidth', 1.5);
grid on; title('Hardware Tracking Error: Y-Axis'); ylabel('Error (mm)'); xlabel('Time (s)');

% Plot the actual path taken vs target path
figure('Name', 'Hardware Cartesian Path', 'Color', 'w');
hold on; grid on; axis equal;
plot(ref_x, ref_y, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Target Path');
plot(actual_pos_log(1,:), actual_pos_log(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Hardware Path');
legend; title('Target vs Physical Path'); xlabel('X (mm)'); ylabel('Y (mm)');