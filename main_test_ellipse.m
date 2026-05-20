% main_test_ellipse.m
% Simulation to test the OpenLoopControl tracking an ellipse trajectory

clear; clc; close all;

%% 1. Setup Parameters & Dimensions
Ts = 0.01;          % Sample time (100 Hz)
T_end = 4;          % Simulation duration (seconds)
t = 0:Ts:T_end;     % Time vector

% Define the robot parameters expected by your kinematics
params = struct();
params.dm.L2 = 100.00; % Thigh
params.dm.L3 = 105.73; % Shin

%% 2. Define the Ellipse Trajectory
% Choosing a center point safely within the reachable workspace
Xc = 0;      % Centered on X
Yc = -150;   % Straight down (Workspace max reach is ~205mm)

% Ellipse dimensions
A = 40;      % Semi-major axis in X (width)
B = 20;      % Semi-minor axis in Y (height)
freq = 0.5;  % 0.5 Hz -> 1 full ellipse every 2 seconds

% Parametric equations for the ellipse
ref_x = Xc + A * cos(2 * pi * freq * t);
ref_y = Yc + B * sin(2 * pi * freq * t);
ref_z = zeros(size(t)); % Planar leg, Z is ignored

%% 3. Initialize Controller
% Instantiate the open loop controller
controller = control.OpenLoopControl(Ts, params);

% WARM START:
% Calculate the IK for the very first point and set it as the PreviousAction.
% If we don't do this, the controller starts at [0;0] and the velocity limiter 
% will take seconds to "catch up" to the first point of the ellipse.
init_ref = [ref_x(1); ref_y(1); ref_z(1)];
init_q = kinematics.inverse_kinematics(init_ref, params);
controller.PreviousAction = init_q;

%% 4. Preallocate Logging Arrays
N = length(t);
action_log = zeros(2, N);
actual_pos_log = zeros(3, N);
ref_pos_log = [ref_x; ref_y; ref_z];

%% 5. Setup Live Animation Figure
fig = figure('Name', 'Live Ellipse Tracking', 'Color', 'w', 'Position', [100, 100, 700, 500]);
hold on; grid on; axis equal;

% Set axis limits slightly larger than the ellipse
xlim([Xc - A - 20, Xc + A + 20]);
ylim([Yc - B - 20, Yc + B + 20]);
xlabel('X Position (mm)', 'FontWeight', 'bold');
ylabel('Y Position (mm)', 'FontWeight', 'bold');
title('Live End-Effector Tracking Simulation');

% Plot the full theoretical reference path in the background
plot(ref_x, ref_y, 'k--', 'Color', [0.7 0.7 0.7], 'DisplayName', 'Reference Path');

% Initialize live plotting handles
h_target = plot(ref_x(1), ref_y(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Target Pos');
h_actual = plot(ref_x(1), ref_y(1), 'b*', 'MarkerSize', 8, 'DisplayName', 'Actual Pos');
h_path   = plot(ref_x(1), ref_y(1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Path');
legend('Location', 'northeast');

%% 6. Simulation Loop
disp('Starting simulation...');
for i = 1:N
    % 1. Get current reference position
    ref = ref_pos_log(:, i);
    
    % 2. Compute Control Action (IK + Velocity Limiter)
    % The empty bracket [] represents the state, which is ignored in open-loop
    action = controller.computeAction([], ref);
    
    % 3. Simulate Robot (Forward Kinematics)
    % In open-loop simulation, we assume perfect tracking of the commanded angle
    actual_pos = kinematics.forward_kinematics(action, params);
    
    % 4. Log Data
    action_log(:, i) = action;
    actual_pos_log(:, i) = actual_pos;
    
    % 5. Update Animation (every step)
    set(h_target, 'XData', ref(1), 'YData', ref(2));
    set(h_actual, 'XData', actual_pos(1), 'YData', actual_pos(2));
    set(h_path, 'XData', actual_pos_log(1, 1:i), 'YData', actual_pos_log(2, 1:i));
    
    % Force MATLAB to draw the graphics immediately
    drawnow; 
end
disp('Simulation complete.');

%% 7. Plot Errors
% Calculate tracking errors
error_x = ref_pos_log(1, :) - actual_pos_log(1, :);
error_y = ref_pos_log(2, :) - actual_pos_log(2, :);

figure('Name', 'Tracking Errors', 'Color', 'w', 'Position', [820, 100, 600, 500]);

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