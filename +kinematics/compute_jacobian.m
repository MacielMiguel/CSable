function J = compute_jacobian(q, params)
    % COMPUTE_JACOBIAN Computes the 2x2 analytical Jacobian matrix numerically
    % using the Central Finite Difference method.
    % 
    % Inputs:
    %   q      - Current joint angles [theta2; a] (radians)
    %   params - Robot parameters struct
    % Output:
    %   J      - 2x2 Jacobian matrix [dx/dtheta2, dx/da; 
    %                                 dy/dtheta2, dy/da]

    % Microscopic perturbation step (1e-5 radians is standard for double precision)
    delta = 1e-5; 
    
    % Initialize 2x2 Jacobian matrix
    J = zeros(2, 2);
    
    %% COLUMN 1: Partial derivatives with respect to theta2 (Motor 1)
    % Perturb theta2 forward and backward
    q_plus_1  = [q(1) + delta; q(2)];
    q_minus_1 = [q(1) - delta; q(2)];
    
    % Get 3D Cartesian positions
    pos_plus_1  = kinematics.forward_kinematics(q_plus_1, params);
    pos_minus_1 = kinematics.forward_kinematics(q_minus_1, params);
    
    % Check for kinematic lockup (NaN) at the workspace boundary
    if any(isnan(pos_plus_1)) || any(isnan(pos_minus_1))
        warning('Jacobian: Near singularity or workspace limit on Motor 1.');
        J(:, 1) = [0; 0]; % Output zero force mapping to prevent NaN explosions
    else
        % Central difference equation: (f(x+h) - f(x-h)) / 2h
        % We only take indices 1 and 2 (X and Y coordinates)
        J(:, 1) = (pos_plus_1(1:2) - pos_minus_1(1:2)) / (2 * delta);
    end
    
    %% COLUMN 2: Partial derivatives with respect to 'a' (Motor 2)
    % Perturb 'a' forward and backward
    q_plus_2  = [q(1); q(2) + delta];
    q_minus_2 = [q(1); q(2) - delta];
    
    % Get 3D Cartesian positions
    pos_plus_2  = kinematics.forward_kinematics(q_plus_2, params);
    pos_minus_2 = kinematics.forward_kinematics(q_minus_2, params);
    
    % Check for kinematic lockup
    if any(isnan(pos_plus_2)) || any(isnan(pos_minus_2))
        warning('Jacobian: Near singularity or workspace limit on Motor 2.');
        J(:, 2) = [0; 0];
    else
        % Central difference equation
        J(:, 2) = (pos_plus_2(1:2) - pos_minus_2(1:2)) / (2 * delta);
    end
end