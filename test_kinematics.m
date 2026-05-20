% Adds all project subfolders to the MATLAB Path
addpath(genpath(pwd)); 

disp('Calculating the workspace... please wait.');
robot_parameters = robot_params();

% 1. WORKSPACE GENERATION
% Running with 100 points for high resolution
[X_work, Y_work] = kinematics.calc_workspace(robot_parameters, 100); 

% 2. TRAJECTORY GENERATION (Elliptical walking step)
disp('Generating elliptical foot trajectory...');
num_traj_points = 200;
t = linspace(0, 2*pi, num_traj_points);

% Ellipse parameters (Centered inside the safe workspace zone)
center_x = 40; % mm
center_y = -145; % mm
radius_x = 30;   % mm (Half-stride length)
radius_y = 15;   % mm (Step clearance height)

target_X = center_x + radius_x * cos(t);
target_Y = center_y + radius_y * sin(t);

% 3. KINEMATIC VALIDATION LOOP (Target -> IK -> FK)
disp('Running Inverse Kinematics (IK) and validating with Forward Kinematics (FK)...');
reconstructed_X = zeros(1, num_traj_points);
reconstructed_Y = zeros(1, num_traj_points);
calc_errors = zeros(1, num_traj_points);

for i = 1:num_traj_points
    target_pos = [target_X(i); target_Y(i); 0];
    
    % Step A: Feed target Cartesian position to Inverse Kinematics
    q_calc = kinematics.inverse_kinematics(target_pos, robot_parameters);
    
    % Safety check for unreachable points
    if any(isnan(q_calc))
        warning('Trajectory point %d is out of reach or in a singular configuration!', i);
        reconstructed_X(i) = NaN;
        reconstructed_Y(i) = NaN;
        continue;
    end
    
    % Step B: Feed the calculated joint angles back to Forward Kinematics
    pos_reconstructed = kinematics.forward_kinematics(q_calc, robot_parameters);
    
    % Store the reconstructed position
    reconstructed_X(i) = pos_reconstructed(1);
    reconstructed_Y(i) = pos_reconstructed(2);
    
    % Calculate the Euclidean error between Target and Reconstructed
    calc_errors(i) = norm([target_X(i) - reconstructed_X(i), target_Y(i) - reconstructed_Y(i)]);
end

% 4. PLOTTING AND VISUALIZATION
% Get the current active figure (from calc_workspace) and hold it
figure(gcf);
hold on;

% Plot the Original Target Trajectory (Black Dashed Line)
plot(target_X, target_Y, 'k--', 'LineWidth', 2, 'DisplayName', 'Target Trajectory');

% Plot the Reconstructed Trajectory (Red Stars)
plot(reconstructed_X, reconstructed_Y, 'r*', 'MarkerSize', 6, 'DisplayName', 'Reconstructed (IK \rightarrow FK)');

% Format the legend and title
legend('Location', 'best');
title('Workspace and Kinematics Validation Loop');
drawnow; % Command that forces MATLAB to render the plot window immediately

disp('Plot generated successfully!');

% 5. NUMERICAL VALIDATION RESULTS
% The error should be extremely close to zero (e.g., 1e-14) if both models are perfect
mean_error = mean(calc_errors(~isnan(calc_errors)));
fprintf('==================================================\n');
fprintf('VALIDATION COMPLETE\n');
fprintf('Mean Reconstruction Error: %.4e mm\n', mean_error);
fprintf('==================================================\n');

% Optional: Display a warning if the error is larger than 0.1 mm
if mean_error > 0.1
    warning('The error is too high! Check the assembly branches in the IK.');
else
    disp('Success! IK and FK match perfectly.');
end