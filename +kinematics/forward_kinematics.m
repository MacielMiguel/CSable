function pos = forward_kinematics(q, params)
    % FORWARD_KINEMATICS Computes the forward kinematics of the² leg
    
    % Unpacking state data
    theta2 = q(1);
    a = q(2);
    
    % Constant intermediate angles
    o = acos(0.6835);
    j = acos(0.0707);
    
    % Clamping the acos for safety
    clamp = @(v) max(min(v, 1), -1);
    
    % Bissection solver to the value of 'a3'
    low = 0;
    high = pi;
    for iter = 1:20
        mid = (low + high)/2;
        k_temp = mid + pi/2 + o - a;
        
        m2_sq = 28.07^2 + 45.95^2 - 2 * 28.07 * 45.95 * cos(k_temp);
        b = acos(clamp((1215.69 - m2_sq) / 1145.30));
        
        m3_sq = 45.95^2 + 20.1^2 - 2 * 45.95 * 20.1 * cos(mid);
        c = acos(clamp((1599.61 - m3_sq) / 1599.43));
        
        % Residue equation: F(a3) = 0
        error = b - c - (pi/2 + o - a);
        if error > 0
            low = mid;
        else
            high = mid;
        end
    end
    
    % KINEMATIC SAFETY LOCK (Kills ghost points)
    if abs(error) > 0.05 % Tolerance
        pos = [NaN; NaN; NaN]; % Returns NaN if the pose is physically impossible
        return;
    end
    
    % Solving the inferior loop
    a3 = low;
    k = a3 + pi/2 + o - a;
    
    % Solving the superior loop
    i_ang = pi/2 - k;
    h = j - i_ang;
    g = theta2 - h;
    
    m1_sq = 10787.92 - 5588 * cos(g);
    m1 = sqrt(m1_sq);
    
    % Calculating the theta3 angle (Relative angle of the shin)
    theta3 = acos(clamp((m1_sq - 9256.35)/5588)) + ...
             acos(clamp((m1_sq + 9219.36)/(200*m1)));
             
    % Unpacking dimensions
    L_thigh = params.dm.L2;    % Thigh (e.g., 100 mm)
    L_shin = params.dm.L3;     % Shin (e.g., 100 mm)
    
    % FINAL CARTESIAN MAPPING (Rotated to the chassis) 
    % Aligning axis 0 with the mechanics (theta2=180 points downwards)
    thigh_angle = theta2 + pi/2; 
    
    % Absolute angle of the shin.
    shin_angle = thigh_angle - theta3; 
    
    x = L_thigh * cos(thigh_angle) + L_shin * cos(shin_angle);
    y = L_thigh * sin(thigh_angle) + L_shin * sin(shin_angle);
    z = 0;
    
    pos = [x; y; z];
end