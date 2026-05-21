% main_test_ellipse_vmc.m
% Dynamic simulation testing the Virtual Model Control (VMC) tracking an ellipse.

clear; clc; close all;

%% 1. Setup Parameters & Dimensions
Ts = 0.01;          % Sample time (100 Hz)
T_end = 4;          % Simulation duration (seconds)
t = 0:Ts:T_end;     % Time vector

% Robot Kinematics Parameters
params = struct();
params.dm.L2 = 100.00; % Thigh
params.dm.L3 = 105.73; % Shin

%% 2. Setup the Simulated Plant (Physics Model)
% We model the leg as a point mass at the foot to simulate acceleration from torque.
foot_mass = 1.0; % [kg] Simulated mass of the leg/environment
current_vel = [0; 0]; % Initial velocity [Vx; Vy]

%% 3. Define the Safe 2D Ellipse Trajectory
Xc = 40;     
Yc = -145;   
A = 20;      % Width
B = 10;      % Height
freq = 0.5;  

% Target positions
ref_x = Xc + A * cos(2 * pi * freq * t);
ref_y = Yc + B * sin(2 * pi * freq * t);

%% 4. Initialize Controller & Warm Start
controller = control.VirtualModelControl(Ts, params);

% Start the physical foot exactly at the beginning of the trajectory
% This prevents the virtual spring from generating a massive snap force on frame 1.
current_pos = [ref_x(1); ref_y(1)];

% Verify the starting point is mathematically valid
init_q = kinematics.inverse_kinematics(current_pos, params);
if any(isnan(init_q))
    error('WORKSPACE ERROR: The starting point is mathematically unreachable.');
end

%% 5. Preallocate Logging Arrays
N = length(t);
tau_log = zeros(2, N);
actual_pos_log = zeros(2, N); 
ref_pos_log = [ref_x; ref_y];   

%% 6. Setup Live Animation Figure
fig = figure('Name', 'Live VMC Ellipse Tracking', 'Color', 'w', 'Position', [100, 100, 700, 500]);
hold on; grid on; axis equal;

xlim([Xc - A - 40, Xc + A + 40]);
ylim([Yc - B - 40, Yc + B + 40]);
xlabel('X Position (mm)', 'FontWeight', 'bold');
ylabel('Y Position (mm)', 'FontWeight', 'bold');
title('Dynamic VMC Tracking Simulation (Spring-Mass)');

plot(ref_x, ref_y, 'k--', 'Color', [0.7 0.7 0.7], 'DisplayName', 'Reference Path');

h_target = plot(ref_x(1), ref_y(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Target Pos');
h_actual = plot(ref_x(1), ref_y(1), 'b*', 'MarkerSize', 8, 'DisplayName', 'Actual Pos');
h_path   = plot(ref_x(1), ref_y(1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Path');
legend('Location', 'northeast');

%% 7. Simulation Loop (Physics Engine + Control)
disp('Starting dynamic VMC simulation...');
for i = 1:N
    % 1. Get current reference target
    ref = ref_pos_log(:, i);
    
    % 2. Calculate the joint angles needed by the Jacobian based on where the foot is
    q_current = kinematics.inverse_kinematics(current_pos, params);
    
    % Safety catch: If the virtual spring pushes the leg out of the workspace
    if any(isnan(q_current))
        warning('Simulated foot was pushed out of the reachable workspace! Stopping simulation.');
        break;
    end
    
    % 3. Construct the state packet for the VMC Controller
    state = struct();
    state.pos = current_pos;
    state.vel = current_vel;
    state.angles = q_current;
    
    % 4. Compute Torque Action [Nm]
    tau = controller.computeAction(state, ref);
    
    % -------------------------------------------------------------
    % 5. SIMULATE PHYSICS (The Plant)
    % -------------------------------------------------------------
    % Get the current numerical Jacobian
    J = kinematics.compute_jacobian(q_current, params);
    
    % The motors apply torque. We convert that torque back into the resulting 
    % Cartesian push/pull force on the foot. (F = J_transpose \ tau)
    % We use pinv (pseudo-inverse) to prevent crashes if it gets near a singularity.
    F_applied = pinv(J') * tau; 
    
    % Newton's Second Law: a = F / m
    % Note: Since dimensions are mm, acceleration scales. We multiply by 1000 
    % to convert Nm and kg roughly into mm/s^2 for the visual simulation.
    accel = (F_applied / foot_mass) * 1000;
    
    % Symplectic Euler Integration (Velocity updates first, then Position)
    % This specific integration method is highly stable for virtual spring systems.
    current_vel = current_vel + accel * Ts;
    current_pos = current_pos + current_vel * Ts;
    
    % 6. Log and Animate Data
    tau_log(:, i) = tau;
    actual_pos_log(:, i) = current_pos;
    
    set(h_target, 'XData', ref(1), 'YData', ref(2));
    set(h_actual, 'XData', current_pos(1), 'YData', current_pos(2));
    set(h_path, 'XData', actual_pos_log(1, 1:i), 'YData', actual_pos_log(2, 1:i));
    
    drawnow; 
end
disp('Simulation complete.');

%% 8. Plot Errors and Output Torques
error_x = ref_pos_log(1, :) - actual_pos_log(1, :);
error_y = ref_pos_log(2, :) - actual_pos_log(2, :);

figure('Name', 'VMC Performance Data', 'Color', 'w', 'Position', [820, 100, 700, 700]);

% Tracking Errors
subplot(3,1,1);
plot(t, error_x, 'r', 'LineWidth', 1.5); hold on;
plot(t, error_y, 'b', 'LineWidth', 1.5);
grid on; title('Tracking Errors'); ylabel('Error (mm)'); legend('X Axis', 'Y Axis');

% Torques
subplot(3,1,2);
plot(t, tau_log(1, :), 'r', 'LineWidth', 1.5);
grid on; title('Motor 1 (Thigh) Commanded Torque'); ylabel('Torque (Nm)');

subplot(3,1,3);
plot(t, tau_log(2, :), 'b', 'LineWidth', 1.5);
grid on; title('Motor 2 (Crank) Commanded Torque'); ylabel('Torque (Nm)'); xlabel('Time (s)');