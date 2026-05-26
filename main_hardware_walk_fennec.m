% main_hardware_walk_fennec.m
% Synchronized dual-leg control using Fennec trajectory with Workspace validation.

clear; clc; close all;

%% 1. Setup Kinematics Parameters & Dimensions
Ts = 0.02;          % Sample time (50 Hz)
T_end = 10;         % Execution duration (10 seconds)
t = 0:Ts:T_end;     % Time vector

params = struct();
params.dm.L2 = 100.00; 
params.dm.L3 = 105.73; 

% FIXED: Added a_offset to the params struct to prevent MATLAB crashes 
% in your updated IK and FK files! Adjust this to your true geometric offset.
params.a_offset = 0; 

% Fennec Data File
ANIMAL_CSV = 'fennec_FINAL_EDITED_foot_cycle.csv';
if ~isfile(ANIMAL_CSV)
    error(['Cannot find ' ANIMAL_CSV '. Run the Fennec trajectory script first!']);
end

%% 2. Setup Hardware Parameters
hw_params = struct();
hw_params.DEVICENAME = 'COM5';          
hw_params.BAUDRATE = 1000000;           
hw_params.PROTOCOL_VERSION = 2.0;       

% [Right Thigh, Right Crank, Left Thigh, Left Crank]
hw_params.DXL_IDS = [4, 3, 2, 1];     
hw_params.DIRECTIONS = [1, 1, -1, -1]; 
hw_params.OFFSETS = [deg2rad(0), deg2rad(143), deg2rad(360), deg2rad(324)];

%% 3. Generate the Biological Trajectory
% Safely placed in the center-bottom of standard parallel leg workspaces
Xc = 0;      
Yc = -140;   
A = 20;      % Half-width (40mm stride)
B = 15;      % Half-height (30mm lift)
freq = 0.5;  
T_cycle = 1 / freq;

t_c_right = mod(t, T_cycle);
t_c_left  = mod(t + (T_cycle / 2), T_cycle); 

disp('Reading Fennec data and mapping to physical workspace...');
[ref_x_R, ref_y_R] = generate_animal_trajectory(t_c_right, T_cycle, Xc, Yc, A, B, ANIMAL_CSV);
[ref_x_L, ref_y_L] = generate_animal_trajectory(t_c_left,  T_cycle, Xc, Yc, A, B, ANIMAL_CSV);

ref_pos_R = [ref_x_R; ref_y_R];
ref_pos_L = [ref_x_L; ref_y_L];

%% 4. PRE-FLIGHT WORKSPACE VALIDATION (Visual Safety Check)
disp('Generating Workspace Boundaries...');
% Run your provided workspace function
[X_ws, Y_ws] = kinematics.calc_workspace(params, 40); 

% The calc_workspace function generates a figure. Let's grab it and add our path.
fig_ws = gcf;
set(fig_ws, 'Name', 'Pre-Flight Workspace Validation', 'Position', [100, 100, 700, 600]);
hold on;

% Overlay the Fennec trajectory
plot(ref_x_R, ref_y_R, 'r-', 'LineWidth', 2.5, 'DisplayName', 'Fennec Path');
plot(ref_x_R(1), ref_y_R(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start Point');
legend('Location', 'best');

% Force the user to visually confirm before moving hardware
disp(' ');
disp('======================================================');
disp('LOOK AT THE PLOT!');
disp('Ensure the red Fennec trajectory is entirely inside the blue workspace bounds.');
disp('If it touches the edge, the mathematical linkages will break (NaN).');
disp('======================================================');
disp(' ');
input('Press ENTER if the trajectory is safe to continue... (or Ctrl+C to abort)');

%% 5. Initialize Controllers & Warm Start
ctrl_R = control.OpenLoopControl(Ts, params);
ctrl_L = control.OpenLoopControl(Ts, params);

init_q_R = kinematics.inverse_kinematics(ref_pos_R(:, 1), params);
init_q_L = kinematics.inverse_kinematics(ref_pos_L(:, 1), params);

if any(isnan(init_q_R)) || any(isnan(init_q_L))
    error('HARD LOCKUP: The trajectory is out of bounds. Adjust Xc, Yc, A, or B.');
end

ctrl_R.PreviousAction = init_q_R;
ctrl_L.PreviousAction = init_q_L;

disp('Initializing hardware connection...');
hw_interface = hardware.DynamixelInterface(hw_params);
hw_interface.init();

init_hw_R = (init_q_R .* hw_params.DIRECTIONS(1:2)') + hw_params.OFFSETS(1:2)';
init_hw_L = (init_q_L .* hw_params.DIRECTIONS(3:4)') + hw_params.OFFSETS(3:4)';

disp('Moving to start positions...');
hw_interface.writePosition(hw_params.DXL_IDS, [init_hw_R; init_hw_L]);
pause(2.0); 

%% 6. REAL-TIME HARDWARE CONTROL LOOP
N = length(t);
actual_pos_R_log = zeros(2, N); 
actual_pos_L_log = zeros(2, N); 

disp('Starting Fennec walk cycle...');

for i = 1:N
    loop_start = tic; 
    
    % Compute Math
    action_R = ctrl_R.computeAction([], ref_pos_R(:, i));
    action_L = ctrl_L.computeAction([], ref_pos_L(:, i));
    
    % Map to Hardware
    hw_cmd_R = (action_R .* hw_params.DIRECTIONS(1:2)') + hw_params.OFFSETS(1:2)';
    hw_cmd_L = (action_L .* hw_params.DIRECTIONS(3:4)') + hw_params.OFFSETS(3:4)';
    
    % Execute
    hw_interface.writePosition(hw_params.DXL_IDS, [hw_cmd_R; hw_cmd_L]);
    
    % Read & Reverse Map
    act_angles_hw = zeros(4,1);
    for m = 1:4
        act_angles_hw(m) = hw_interface.readPosition(hw_params.DXL_IDS(m));
    end
    
    act_q_R = (act_angles_hw(1:2) - hw_params.OFFSETS(1:2)') ./ hw_params.DIRECTIONS(1:2)';
    act_q_L = (act_angles_hw(3:4) - hw_params.OFFSETS(3:4)') ./ hw_params.DIRECTIONS(3:4)';
    
    % Log Real Cartesian Physics
    act_pos_3d_R = kinematics.forward_kinematics(act_q_R, params);
    act_pos_3d_L = kinematics.forward_kinematics(act_q_L, params);
    
    actual_pos_R_log(:, i) = act_pos_3d_R(1:2);
    actual_pos_L_log(:, i) = act_pos_3d_L(1:2);
    
    % Pacing
    elapsed_time = toc(loop_start);
    if elapsed_time < Ts
        pause(Ts - elapsed_time);
    end
end

disp('Walking complete. Cleaning up...');
hw_interface.cleanup();

%% 7. Plot Post-Run Tracking
figure('Name', 'Fennec Tracking Performance', 'Color', 'w', 'Position', [100, 100, 800, 400]);

subplot(1,2,1); hold on; grid on; axis equal;
plot(ref_pos_R(1,:), ref_pos_R(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Target');
plot(actual_pos_R_log(1,:), actual_pos_R_log(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
title('Right Leg Tracking'); xlabel('X (mm)'); ylabel('Y (mm)'); legend;

subplot(1,2,2); hold on; grid on; axis equal;
plot(ref_pos_L(1,:), ref_pos_L(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Target');
plot(actual_pos_L_log(1,:), actual_pos_L_log(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
title('Left Leg Tracking'); xlabel('X (mm)'); ylabel('Y (mm)'); legend;

%% =========================================================
%% LOCAL HELPER FUNCTIONS
%% =========================================================
function [ref_x, ref_y] = generate_animal_trajectory(t_c, T_cycle, Xc, Yc, A, B, csv_file)
    % Reads biological data and normalizes it to the physical robot workspace.
    data = readtable(csv_file);
    x_raw = data.x_px;
    z_raw = data.z_px;

    % Normalize
    x_norm = (x_raw - min(x_raw)) / (max(x_raw) - min(x_raw));
    
    z_range = max(z_raw) - min(z_raw);
    if z_range == 0, z_norm = zeros(size(z_raw));
    else, z_norm = (z_raw - min(z_raw)) / z_range; end

    % Map to Robot constraints
    x_mapped = (Xc - A) + x_norm * (2 * A);
    y_mapped = (Yc - B) + z_norm * (2 * B);

    % Interpolate
    t_csv = linspace(0, T_cycle, length(x_mapped))';
    ref_x = interp1(t_csv, x_mapped, t_c, 'pchip', 'extrap');
    ref_y = interp1(t_csv, y_mapped, t_c, 'pchip', 'extrap');
end