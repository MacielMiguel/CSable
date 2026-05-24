% main_hardware_walk.m
% Real-time synchronization of two front legs tracking a cycloid gait.

clear; clc; close all;

%% 1. Setup Kinematics Parameters & Dimensions
Ts = 0.02;          % Sample time (50 Hz) - SLOWED DOWN for 4-motor serial traffic
T_end = 10;         % Execution duration (10 seconds of walking)
t = 0:Ts:T_end;     % Time vector

params = struct();
params.dm.L2 = 100.00; 
params.dm.L3 = 105.73; 

%% 2. Setup Combined Hardware Parameters
hw_params = struct();
hw_params.DEVICENAME = 'COM5';          % Check your COM port
hw_params.BAUDRATE = 1000000;           
hw_params.PROTOCOL_VERSION = 2.0;       

% --- UPDATE THESE IDs TO MATCH YOUR PHYSICAL SETUP ---
% [Right Thigh, Right Crank, Left Thigh, Left Crank]
hw_params.DXL_IDS = [4, 3, 2, 1];     

% Directions and Offsets combined for all 4 motors
hw_params.DIRECTIONS = [1, 1, -1, -1]; 
hw_params.OFFSETS = [deg2rad(0), deg2rad(143), deg2rad(360), deg2rad(324)];

%% 3. Generate Synchronized Cycloid Trajectories
Xc = 40;     
Yc = -145;   
A = 20;      
B = 15;      
freq = 0.5;  
T_cycle = 1 / freq;

% Phase Shift: Right starts at 0, Left starts halfway through the cycle
t_c_right = mod(t, T_cycle);
t_c_left  = mod(t + (T_cycle / 2), T_cycle); 

[ref_x_R, ref_y_R] = generate_cycloid(t_c_right, T_cycle, Xc, Yc, A, B);
[ref_x_L, ref_y_L] = generate_cycloid(t_c_left,  T_cycle, Xc, Yc, A, B);

ref_pos_R = [ref_x_R; ref_y_R];
ref_pos_L = [ref_x_L; ref_y_L];

%% 4. Initialize Subsystems
% Create TWO independent controllers (one for each leg)
ctrl_R = control.OpenLoopControl(Ts, params);
ctrl_L = control.OpenLoopControl(Ts, params);

% Calculate initial positions
init_q_R = kinematics.inverse_kinematics(ref_pos_R(:, 1), params);
init_q_L = kinematics.inverse_kinematics(ref_pos_L(:, 1), params);

if any(isnan(init_q_R)) || any(isnan(init_q_L))
    error('WORKSPACE ERROR: One of the starting points is unreachable.');
end

ctrl_R.PreviousAction = init_q_R;
ctrl_L.PreviousAction = init_q_L;

% Initialize Hardware
disp('Initializing hardware connection...');
hw_interface = hardware.DynamixelInterface(hw_params);
hw_interface.init();

% --- MAP KINEMATICS TO HARDWARE SPACE FOR WARM START ---
init_hw_R = (init_q_R .* hw_params.DIRECTIONS(1:2)') + hw_params.OFFSETS(1:2)';
init_hw_L = (init_q_L .* hw_params.DIRECTIONS(3:4)') + hw_params.OFFSETS(3:4)';

disp('Moving to synchronized start positions. Please stand clear...');
hw_interface.writePosition(hw_params.DXL_IDS, [init_hw_R; init_hw_L]);
pause(2.0); % Wait for legs to lock into starting stance

%% 5. Preallocate Logging
N = length(t);
actual_pos_R_log = zeros(2, N); 
actual_pos_L_log = zeros(2, N); 

disp('System Hot! Starting dual-leg walking cycle...');

%% 6. REAL-TIME HARDWARE CONTROL LOOP
for i = 1:N
    loop_start = tic; 
    
    % 1. Compute Control Actions (Pure Math Space)
    action_R = ctrl_R.computeAction([], ref_pos_R(:, i));
    action_L = ctrl_L.computeAction([], ref_pos_L(:, i));
    
    % 2. Map Actions to Hardware Space (Apply Directions & Offsets)
    hw_cmd_R = (action_R .* hw_params.DIRECTIONS(1:2)') + hw_params.OFFSETS(1:2)';
    hw_cmd_L = (action_L .* hw_params.DIRECTIONS(3:4)') + hw_params.OFFSETS(3:4)';
    
    % 3. Send all 4 commands in one hardware call
    hw_interface.writePosition(hw_params.DXL_IDS, [hw_cmd_R; hw_cmd_L]);
    
    % 4. Read Hardware and Map Back to Math Space
    act_angles_hw = zeros(4,1);
    for m = 1:4
        act_angles_hw(m) = hw_interface.readPosition(hw_params.DXL_IDS(m));
    end
    
    % Reverse the hardware mapping for the Forward Kinematics
    act_q_R = (act_angles_hw(1:2) - hw_params.OFFSETS(1:2)') ./ hw_params.DIRECTIONS(1:2)';
    act_q_L = (act_angles_hw(3:4) - hw_params.OFFSETS(3:4)') ./ hw_params.DIRECTIONS(3:4)';
    
    act_pos_3d_R = kinematics.forward_kinematics(act_q_R, params);
    act_pos_3d_L = kinematics.forward_kinematics(act_q_L, params);
    
    % 5. Log Data
    actual_pos_R_log(:, i) = act_pos_3d_R(1:2);
    actual_pos_L_log(:, i) = act_pos_3d_L(1:2);
    
    % 6. Pacing constraint
    elapsed_time = toc(loop_start);
    if elapsed_time < Ts
        pause(Ts - elapsed_time);
    end
end

disp('Walking complete. Cleaning up...');
hw_interface.cleanup();

%% 7. Plot Synchronized Tracking
figure('Name', 'Dual Leg Synchronization', 'Color', 'w', 'Position', [100, 100, 800, 400]);

subplot(1,2,1); hold on; grid on; axis equal;
plot(ref_pos_R(1,:), ref_pos_R(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Target');
plot(actual_pos_R_log(1,:), actual_pos_R_log(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
title('Right Leg Path'); xlabel('X (mm)'); ylabel('Y (mm)'); legend;

subplot(1,2,2); hold on; grid on; axis equal;
plot(ref_pos_L(1,:), ref_pos_L(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Target');
plot(actual_pos_L_log(1,:), actual_pos_L_log(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
title('Left Leg Path'); xlabel('X (mm)'); ylabel('Y (mm)'); legend;

%% =========================================================
%% LOCAL HELPER FUNCTIONS
%% =========================================================
function [ref_x, ref_y] = generate_cycloid(t_c, T_cycle, Xc, Yc, A, B)
    % Generates a perfectly timed cycloid based on the provided phase time
    ref_x = zeros(size(t_c));
    ref_y = zeros(size(t_c));
    
    swing_mask = t_c < (T_cycle / 2);
    stance_mask = ~swing_mask;

    % Swing Phase (Air)
    tau_swing = t_c(swing_mask) / (T_cycle / 2); 
    ref_x(swing_mask) = (Xc - A) + (2*A / (2*pi)) * (2*pi*tau_swing - sin(2*pi*tau_swing));
    ref_y(swing_mask) = (Yc - B) + B * (1 - cos(2*pi*tau_swing));

    % Stance Phase (Ground)
    tau_stance = (t_c(stance_mask) - (T_cycle / 2)) / (T_cycle / 2); 
    ref_x(stance_mask) = (Xc + A) - (2*A * tau_stance);
    ref_y(stance_mask) = (Yc - B); 
end