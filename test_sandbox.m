% test_sandbox.m
% A pure software test to validate Open Loop and Virtual Model Control logic.

clear all; clc; close all;

%% 1. Setup Mock Parameters (Simple 2-link leg)
Ts = 0.01;
params.L1 = 0.1; % 10 cm thigh
params.L2 = 0.1; % 10 cm calf

%% 2. Instantiate the Controllers
disp('--- Initializing Controllers ---');
ctrl_OL = control.OpenLoopControl(Ts, params);
ctrl_VMC = control.VirtualModelControl(Ts, params);


%% 3. Define a Mock State and Target
% Let's pretend the leg is currently hanging straight down
state.angles = [-pi/2; 0]; 
state.pos = kinematics.forward_kinematics(state.angles, params); 
state.vel = [0; 0];

% We want the foot to move slightly forward and up
ref = [0.05; -0.15]; 

% ---> ADD THIS LINE TO SYNC THE CONTROLLER'S MEMORY <---
ctrl_OL.PreviousAction = state.angles;
%% 4. Test Open Loop Control (With Time Integration)
disp('--- Testing Open Loop Control ---');


max_steps = 200; 
for i = 1:max_steps
    ol_action = ctrl_OL.computeAction(state, ref);
    % Update the "state" so the controller knows where it is now
    state.angles = ol_action;
end

disp(['Final Angles after 20 steps [rad]: theta1=', num2str(ol_action(1)), ', theta2=', num2str(ol_action(2))]);

% Verify via Forward Kinematics
check_pos = kinematics.forward_kinematics(ol_action, params);
disp(['Final Position Check: X=', num2str(check_pos(1)), ' Z=', num2str(check_pos(2))]);
disp(' ');

%% 5. Test Virtual Model Control
disp('--- Testing Virtual Model Control ---');
vmc_action = ctrl_VMC.computeAction(state, ref);
disp(['Commanded Torques [Nm]: tau1=', num2str(vmc_action(1)), ', tau2=', num2str(vmc_action(2))]);
disp(' ');

%% 6. Visualizing the Result (Open Loop)
figure('Name', 'Controller Sandbox Test', 'Color', 'w');
hold on; grid on; axis equal;
xlim([-0.2 0.2]); ylim([-0.25 0.05]);
title('2-DOF Leg Kinematics Test');
xlabel('X [m]'); ylabel('Z [m]');

% Plot Origin (Hip)
plot(0, 0, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

% Plot Target Reference
plot(ref(1), ref(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Calculate Knee Position from the Open Loop commanded angles
knee_x = params.L1 * cos(ol_action(1));
knee_z = params.L1 * sin(ol_action(1));

% Plot Links
plot([0, knee_x], [0, knee_z], 'b-', 'LineWidth', 3); % Thigh
plot([knee_x, check_pos(1)], [knee_z, check_pos(2)], 'b-', 'LineWidth', 3); % Calf

% Plot Joints
plot(knee_x, knee_z, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y'); % Knee
plot(check_pos(1), check_pos(2), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'g'); % Foot

legend('Hip Joint', 'Target Ref', 'Leg Links', 'Location', 'best');

% --- FORCING THE PLOT TO SHOW ---
drawnow; % Forces MATLAB to render the graphics immediately
shg;     % "Show Graph" - brings the figure window to the front
disp('Execution paused. Press any key in the Command Window to exit...');
pause;   % Halts the script forever until you press any key in the console