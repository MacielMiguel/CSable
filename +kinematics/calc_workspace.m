function [X, Y] = calc_workspace(dm_params, num_points)
    % CALC_WORKSPACE Plots and returns the coordinates of the leg's workspace
    % Inputs:
    %   dm_params  - Struct with the link lengths (L_thigh, L_shin)
    %   num_points - Grid resolution (e.g., 50 generates 2500 calculated points)
    
    if nargin < 2
        num_points = 20; % Default resolution
    end
    
    % Definition of motor limits in radians
    a_vec = linspace(deg2rad(90), deg2rad(130), num_points); % Optimal angles interval [90, 130]
    theta2_vec = linspace(deg2rad(145), deg2rad(270), num_points); % Optimal angles interval [145, 270]
    
    % MEMORY PREALLOCATION FOR EFFICIENCY (O(N^2))
    total_points = num_points * num_points;
    X = zeros(total_points, 1);
    Y = zeros(total_points, 1);
    
    % JOIN SPACE SWEEP
    idx = 1;
    for i = 1:length(theta2_vec)
        for j = 1:length(a_vec)
            % Packing the current state. 
            % Based on your FK: q(1) = theta2, q(2) = a
            q_current = [theta2_vec(i); a_vec(j)];
            
            % Calculates Cartesian position
            pos = kinematics.forward_kinematics(q_current, dm_params);
            
            % Saves X and Y coordinates (discarding Z)
            X(idx) = pos(1);
            Y(idx) = pos(2);
            idx = idx + 1;
        end
    end
    
    % INVALID POINTS CLEANUP (NaN Filter)
    % Creates a logical index: where X is NOT NaN
    valid_idx = ~isnan(X); 
    
    % Keeps only coordinates that passed the physical lock
    X_clean = X(valid_idx);
    Y_clean = Y(valid_idx);
    
    % DATA VISUALIZATION
    figure('Name', 'Workspace Analysis', 'Color', 'white');
    
    % Plots the point cloud using the clean vectors
    scatter(X_clean, Y_clean, 15, 'filled', 'MarkerFaceColor', '#11caa0', 'MarkerFaceAlpha', 0.4);
    hold on;
    
    % Extracts and plots the boundary using the clean vectors
    % Without NaNs, 'boundary' will work perfectly
    k = boundary(X_clean, Y_clean, 0.8);
    plot(X_clean(k), Y_clean(k), 'Color', '#003366', 'LineWidth', 2);
    
    % Marker for the origin (Hip)
    plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    text(5, 5, 'Hip (0,0)', 'FontWeight', 'bold');
    
    % Plot Formatting
    title('Real Reachable Workspace of the Leg');
    xlabel('X Position (mm)');
    ylabel('Y Position (mm)');
    axis equal; 
    grid on;
    set(gca, 'FontSize', 12);
    
    % Returns the clean vectors to the output variable (optional, but recommended)
    X = X_clean;
    Y = Y_clean;
end