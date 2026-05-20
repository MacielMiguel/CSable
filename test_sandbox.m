% test_sandbox.m
clear; clc; close all;

%% 1. Mock Parameters (Simple 2-link leg)
Ts = 0.01;
params.L1 = 0.1; % 10 cm thigh
params.L2 = 0.1; % 10 cm calf

%% 2. Init Controllers
ctrl_OL = control.OpenLoopControl(Ts, params);
ctrl_VMC = control.VirtualModelControl(Ts, params);

%% 3. Mock State & Target
state.angles = [-pi/2; 0]; 
state.pos = kinematics.forward_kinematics(state.angles, params); 
state.vel = [0; 0];
ref = [0.05; -0.15]; % Target position

%% 4. Run Open Loop
ol_action = ctrl_OL.computeAction(state, ref);
check_pos = kinematics.forward_kinematics(ol_action, params);
disp('--- OPEN LOOP ---');
disp(['Calculated Angles: ', num2str(ol_action')]);
disp(['Final Position Check: X=', num2str(check_pos(1)), ' Z=', num2str(check_pos(2))]);

%% 5. Run VMC
vmc_action = ctrl_VMC.computeAction(state, ref);
disp('--- VIRTUAL MODEL CONTROL ---');
disp(['Commanded Torques: ', num2str(vmc_action')]);

%% 6. Plotting
figure('Color', 'w'); hold on; grid on; axis equal;
xlim([-0.2 0.2]); ylim([-0.25 0.05]);
title('Controller Sandbox Test'); xlabel('X [m]'); ylabel('Z [m]');

% Plot Hip and Target
plot(0, 0, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot(ref(1), ref(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Plot Leg
knee_x = params.L1 * cos(ol_action(1));
knee_z = params.L1 * sin(ol_action(1));
plot([0, knee_x], [0, knee_z], 'b-', 'LineWidth', 3);
plot([knee_x, check_pos(1)], [knee_z, check_pos(2)], 'b-', 'LineWidth', 3);

legend('Hip', 'Target', 'Leg', 'Location', 'best');