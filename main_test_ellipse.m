% main_test_ellipse.m
% 2D Simulation to test the OpenLoopControl tracking an ellipse trajectory

clear; clc; close all;

%% 1. Setup Parameters & Dimensions
Ts = 0.01;          % Sample time (100 Hz)
T_end = 4;          % Simulation duration (seconds)
t = 0:Ts:T_end;     % Time vector

% Define the robot parameters expected by your kinematics
params = struct();
params.dm.L2 = 100.00; % Thigh
params.dm.L3 = 105.73; % Shin

%% 2. Define a Safe 2D Ellipse Trajectory
Xc = 40;      % Centered on X
Yc = -145;   % Positioned vertically within mechanical limits
A = 20;      % Ellipse width semi-axis (mm)
B = 10;      % Ellipse height semi-axis (mm)
freq = 0.5;  % 0.5 Hz (1 cycle every 2 seconds)

% Parametric equations for the 2D ellipse
ref_x = Xc + A * cos(2 * pi * freq * t);
ref_y = Yc + B * sin(2 * pi * freq * t);

%% 3. Initialize Controller & Warm Start Check
controller = control.OpenLoopControl(Ts, params);

% 2D Target reference for the initial step
init_ref = [ref_x(1); ref_y(1)];
init_q = kinematics.inverse_kinematics(init_ref, params);

% Workspace check to prevent silent failures
if any(isnan(init_q))
    error(['WORKSPACE ERROR: The starting point (X: %.1f, Y: %.1f) is unreachable. ' ...
           'The internal linkages locked up. Please adjust Xc, Yc, A, or B.'], init_ref(1), init_ref(2));
end

% Set the initial state memory
controller.PreviousAction = init_q;

%% 4. Preallocate 2D Logging Arrays
N = length(t);
action_log = zeros(2, N);
actual_pos_log = zeros(2, N); % 2D Log (X and Y only)
ref_pos_log = [ref_x; ref_y];   % 2D Reference Matrix

%% 5. Setup Live Animation Figure
fig = figure('Name', 'Live Ellipse Tracking (2D)', 'Color', 'w', 'Position', [100, 100, 700, 500]);
hold on; grid on; axis equal;

xlim([Xc - A - 30, Xc + A + 30]);
ylim([Yc - B - 30, Yc + B + 30]);
xlabel('X Position (mm)', 'FontWeight', 'bold');
ylabel('Y Position (mm)', 'FontWeight', 'bold');
title('Live End-Effector 2D Tracking Simulation');

% Plot the full reference path outline
plot(ref_x, ref_y, 'k--', 'Color', [0.7 0.7 0.7], 'DisplayName', 'Reference Path');

% Initialize live plotting handles
h_target = plot(ref_x(1), ref_y(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Target Pos');
h_actual = plot(ref_x(1), ref_y(1), 'b*', 'MarkerSize', 8, 'DisplayName', 'Actual Pos');
h_path   = plot(ref_x(1), ref_y(1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Path');
legend('Location', 'northeast');

%% 6. Simulation Loop
disp('Starting simulation...');
for i = 1:N
    % 1. Get current 2D reference position
    ref = ref_pos_log(:, i);
    
    % 2. Compute Control Action (Returns joint angles [theta2; a])
    action = controller.computeAction([], ref);
    
    % 3. Simulate Robot (Forward Kinematics)
    actual_pos_3d = kinematics.forward_kinematics(action, params);
    actual_pos = actual_pos_3d(1:2); % Truncate Z to keep it strictly 2D
    
    % 4. Log Data
    action_log(:, i) = action;
    actual_pos_log(:, i) = actual_pos;
    
    % 5. Update Animation Live
    set(h_target, 'XData', ref(1), 'YData', ref(2));
    set(h_actual, 'XData', actual_pos(1), 'YData', actual_pos(2));
    set(h_path, 'XData', actual_pos_log(1, 1:i), 'YData', actual_pos_log(2, 1:i));
    
    drawnow; 
end
disp('Simulation complete.');

%% 7. Plot 2D Tracking Errors
error_x = ref_pos_log(1, :) - actual_pos_log(1, :);
error_y = ref_pos_log(2, :) - actual_pos_log(2, :);

figure('Name', '2D Tracking Errors', 'Color', 'w', 'Position', [820, 100, 600, 500]);

subplot(2,1,1);
plot(t, error_x, 'r', 'LineWidth', 1.5);
grid on;
title('Tracking Error in X-Axis');
ylabel('Error (mm)');

subplot(2,1,2);
plot(t, error_y, 'b', 'LineWidth', 1.5);
grid on;
title('Tracking Error in Y-Axis');
ylabel('Error (mm)');
xlabel('Time (s)');