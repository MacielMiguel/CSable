% Adds all project subfolders to the MATLAB Path
addpath(genpath(pwd)); 

disp('Calculating the workspace... please wait.');
robot_parameters = robot_params();

% Running with 20 points first to be faster (3600 iterations)
% Note: The function call below uses 100 points, but keeping the comment translation literal.
[X_work, Y_work] = kinematics.calc_workspace(robot_parameters, 100); 

drawnow; % Command that forces MATLAB to render the plot window immediately
figure(gcf); % Brings the plot window to the front of all others
disp('Plot generated successfully!');