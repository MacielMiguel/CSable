% main_hardware_ellipse.m
% Real-time hardware execution of the OpenLoopControl tracking an ellipse.
% Supports symmetrical configuration for Left and Right legs.

clear; clc; close all;

%% USER CONFIGURATION
% Choose which leg to control. 
% Options: 'RIGHT' or 'LEFT'
TARGET_LEG = 'LEFT'; 

%% Setup Kinematics Parameters & Dimensions
Ts = 0.01;                              % Sample time (100 Hz)
T_end = 4;                              % Execution duration (seconds)
t = 0:Ts:T_end;                         % Time vector

% Robot Kinematics Parameters (Strictly in millimeters)
params = struct();
params.dm.L2 = 100.00;                  % Thigh
params.dm.L3 = 105.73;                  % Shin

%% Setup Hardware Parameters based on Target Leg
hw_params = struct();
hw_params.DEVICENAME = 'COM5';          % Check your COM port
hw_params.BAUDRATE = 1000000;           % Default for X-Series
hw_params.PROTOCOL_VERSION = 2.0;       % X-Series MUST use Protocol 2.0

switch TARGET_LEG
    case 'RIGHT'
        disp('Configuring hardware for RIGHT leg...');
        hw_params.DXL_IDS = [4, 3];     % [Thigh Motor ID, Crank Motor ID]
        hw_params.DIRECTIONS = [1, 1];  % Standard rotation
        
        offset_theta2 = deg2rad(0);   
        offset_a      = deg2rad(143); 
        hw_params.OFFSETS = [offset_theta2, offset_a];
        
    case 'LEFT'
        disp('Configuring hardware for LEFT leg...');
        hw_params.DXL_IDS = [4, 3];    
        hw_params.DIRECTIONS = [-1, -1]; % Inverted rotation for symmetry
        
        offset_theta2 = deg2rad(0);
        offset_a      = deg2rad(-36);
        hw_params.OFFSETS = [offset_theta2, offset_a];
        
    otherwise
        error('Invalid TARGET_LEG. Please select ''RIGHT'' or ''LEFT''.');
end

%% Define the Safe 2D Ellipse Trajectory
% Adjusted to comfortably fit inside the reachable workspace
Xc = 40;     
Yc = -145;   
A = 20;      
B = 15;      
freq = 0.5;                             % 0.5 Hz (1 cycle every 2 seconds)

ref_x = Xc + A * cos(2 * pi * freq * t);
ref_y = Yc + B * sin(2 * pi * freq * t);

%% Workspace and Trajectory Pre-Visualization
disp('Calculating Workspace for safety verification...');
% Uses a resolution of 60 (3600 points) for a good balance of speed and detail
[~, ~] = kinematics.calc_workspace(params, 60); 

% Grab the figure created by calc_workspace to add our trajectory
fig_workspace = gcf;
set(fig_workspace, 'Name', ['Workspace and Cartesian Path - ' TARGET_LEG ' Leg']);
title(['Workspace and Physical Path - ' TARGET_LEG ' Leg']);
hold on;

% Plot the target trajectory
plot(ref_x, ref_y, 'k--', 'LineWidth', 2, 'DisplayName', 'Target Path');
plot(ref_x(1), ref_y(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start Point');
legend('Location', 'best');
drawnow;

%% Initialize Subsystems
% Initialize Controller & Warm Start Check
controller = control.OpenLoopControl(Ts, params);

init_ref = [ref_x(1); ref_y(1)];
init_q = kinematics.inverse_kinematics(init_ref, params);

if any(isnan(init_q))
    error('WORKSPACE ERROR: The starting point is unreachable. Check your Xc, Yc, A, B.');
end
controller.PreviousAction = init_q;

% Initialize Hardware
disp('Initializing hardware connection...');
hw_interface = hardware.DynamixelInterface(hw_params);
hw_interface.init();

% Move to the starting position slowly before the fast trajectory begins
disp('Moving to start position. Please stand clear...');
hw_interface.writePosition(hw_params.DXL_IDS, init_q);
pause(2.0); % Give the motors 2 seconds to reach the start point safely

%% Preallocate Logging Arrays
N = length(t);
action_log = zeros(2, N);
actual_pos_log = zeros(2, N); 
ref_pos_log = [ref_x; ref_y];   

disp(['Starting trajectory execution for ' TARGET_LEG ' leg...']);

%% REAL-TIME HARDWARE CONTROL LOOP
for i = 1:N
    % Mark the exact start time of this loop iteration
    loop_start = tic; 
    
    % Get current reference
    ref = ref_pos_log(:, i);
    
    % Compute Control Action (IK + Limiter)
    action = controller.computeAction([], ref);
    
    % Send to Hardware!
    hw_interface.writePosition(hw_params.DXL_IDS, action);
    
    % Read Actual Hardware Position (for plotting errors)
    act_theta2 = hw_interface.readPosition(hw_params.DXL_IDS(1));
    act_a      = hw_interface.readPosition(hw_params.DXL_IDS(2));
    act_angles = [act_theta2; act_a];
    
    % Calculate the real physical Cartesian location using FK
    act_pos_3d = kinematics.forward_kinematics(act_angles, params);
    actual_pos = act_pos_3d(1:2);
    
    % Log Data
    action_log(:, i) = action;
    actual_pos_log(:, i) = actual_pos;
    
    % Real-time pacing constraint
    elapsed_time = toc(loop_start);
    if elapsed_time < Ts
        pause(Ts - elapsed_time);
    end
end

disp('Trajectory complete. Cleaning up hardware...');

%% Safe Cleanup
hw_interface.cleanup();

%% Plot Final Hardware Tracking Errors
error_x = ref_pos_log(1, :) - actual_pos_log(1, :);
error_y = ref_pos_log(2, :) - actual_pos_log(2, :);

% Plot the time-domain tracking errors
figure('Name', ['Tracking Errors - ' TARGET_LEG ' Leg'], 'Color', 'w');
subplot(2,1,1);
plot(t, error_x, 'r', 'LineWidth', 1.5);
grid on; title(['Hardware Tracking Error (X-Axis) - ' TARGET_LEG]); ylabel('Error (mm)');

subplot(2,1,2);
plot(t, error_y, 'b', 'LineWidth', 1.5);
grid on; title(['Hardware Tracking Error (Y-Axis) - ' TARGET_LEG]); ylabel('Error (mm)'); xlabel('Time (s)');

% Overlay the Actual Hardware Path on the previously generated Workspace plot
figure(fig_workspace); 
plot(actual_pos_log(1,:), actual_pos_log(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Actual Hardware Path');
legend('Location', 'best');