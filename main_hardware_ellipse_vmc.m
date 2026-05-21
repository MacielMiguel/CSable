% main_hardware_ellipse_vmc.m
% Real-time hardware execution of Virtual Model Control (VMC) tracking an ellipse.

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
hw_params.DEVICENAME = 'COM3';          % Windows: 'COM3', Linux: '/dev/ttyUSB0'
hw_params.BAUDRATE = 1000000;           
hw_params.PROTOCOL_VERSION = 2.0;       
hw_params.DXL_IDS = [1, 2];             % [Thigh Motor ID, Crank Motor ID]

%% 3. Define the 2D Ellipse Trajectory
Xc = 40;     
Yc = -145;   
A = 20;      
B = 10;      
freq = 0.5;  

ref_x = Xc + A * cos(2 * pi * freq * t);
ref_y = Yc + B * sin(2 * pi * freq * t);
ref_pos_log = [ref_x; ref_y];   

%% 4. Initialize Subsystems
% 4a. Initialize VMC Controller
controller = control.VirtualModelControl(Ts, params);

% 4b. Initialize Hardware Connection
disp('Initializing hardware connection...');
% Using the dedicated VMC interface. This handles Current Mode switching automatically!
hw_interface = hardware.DynamixelInterfaceVMC(hw_params);
hw_interface.init();

%% 5. Pre-Positioning & Seed Variables
% Read the initial position to calculate velocity on the first loop frame
init_theta2 = hw_interface.readPosition(hw_params.DXL_IDS(1));
init_a      = hw_interface.readPosition(hw_params.DXL_IDS(2));
init_pos_3d = kinematics.forward_kinematics([init_theta2; init_a], params);

prev_pos = init_pos_3d(1:2); % Seed value for velocity derivative
N = length(t);

% Preallocate logging arrays
tau_log = zeros(2, N);
actual_pos_log = zeros(2, N); 

% --- SAFETY COUNTDOWN ---
disp(' ');
disp('WARNING: Torque Control turning on.');
disp('Please hold the leg near the starting position to prevent snapping!');
for c = 3:-1:1
    fprintf('%d...\n', c);
    pause(1.0);
end
disp('System Hot! Starting real-time VMC loop...');

%% 6. REAL-TIME HARDWARE VMC LOOP
for i = 1:N
    loop_start = tic; 
    
    % 1. Get current tracking reference
    ref = ref_pos_log(:, i);
    
    % 2. Read current physical joint states
    act_theta2 = hw_interface.readPosition(hw_params.DXL_IDS(1));
    act_a      = hw_interface.readPosition(hw_params.DXL_IDS(2));
    current_angles = [act_theta2; act_a];
    
    % 3. Calculate physical Foot position (FK)
    act_pos_3d = kinematics.forward_kinematics(current_angles, params);
    current_pos = act_pos_3d(1:2);
    
    % 4. Derive physical Foot velocity (Discrete Backward Difference)
    current_vel = (current_pos - prev_pos) / Ts;
    prev_pos = current_pos; % Update history for next iteration
    
    % 5. Package state and calculate VMC Torque
    state = struct();
    state.pos = current_pos;
    state.vel = current_vel;
    state.angles = current_angles;
    
    tau = controller.computeAction(state, ref);
    
    % 6. Send Torques to Hardware
    hw_interface.writeTorque(hw_params.DXL_IDS, tau);
    
    % 7. Log Data
    tau_log(:, i) = tau;
    actual_pos_log(:, i) = current_pos;
    
    % 8. Real-time loop pacing
    elapsed_time = toc(loop_start);
    if elapsed_time < Ts
        pause(Ts - elapsed_time);
    end
end

disp('Trajectory complete. Relaxing motors...');

%% 7. Safe Cleanup
hw_interface.cleanup();

%% 8. Plot Post-Run Performance Graphics
error_x = ref_pos_log(1, :) - actual_pos_log(1, :);
error_y = ref_pos_log(2, :) - actual_pos_log(2, :);

figure('Name', 'Hardware VMC Performance', 'Color', 'w');
subplot(3,1,1);
plot(t, error_x, 'r', t, error_y, 'b', 'LineWidth', 1.5);
grid on; title('Hardware Tracking Errors'); ylabel('Error (mm)'); legend('X Axis', 'Y Axis');

subplot(3,1,2);
plot(t, tau_log(1, :), 'r', 'LineWidth', 1.5);
grid on; title('Motor 1 (Thigh) Applied Torque'); ylabel('Torque (Nm)');

subplot(3,1,3);
plot(t, tau_log(2, :), 'b', 'LineWidth', 1.5);
grid on; title('Motor 2 (Crank) Applied Torque'); ylabel('Torque (Nm)'); xlabel('Time (s)');

figure('Name', 'Physical Space Path', 'Color', 'w');
hold on; grid on; axis equal;
plot(ref_x, ref_y, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Target Ellipse');
plot(actual_pos_log(1,:), actual_pos_log(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Foot Path');
legend; title('Physical VMC Trajectory Mapping'); xlabel('X (mm)'); ylabel('Y (mm)');