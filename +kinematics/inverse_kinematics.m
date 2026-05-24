function q = inverse_kinematics(pos, params)
    % INVERSE_KINEMATICS Computes the motor angles for a given foot position
    % Inputs:
    %   pos - Target Cartesian coordinates [x; y; z] (z is ignored)
    %   params - Robot parameters struct containing nested 'dm' dimensions
    % Output:
    %   q - Required joint angles vector [theta2; a] (radians)
    
    x = pos(1);
    y = pos(2);
    
    % =========================================================
    % LINK LENGTHS (Must match the FK exactly)
    % =========================================================
    AD = 50; 
    AB = 20.10; 
    BC = 29.49; 
    CD = 28.07; 
    DE = 27.94; 
    CE = 38.18; 
    EF = 100.00; 
    FG = 27.27; 
    
    DG = params.dm.L2; % Thigh (100.00 mm)
    GH = params.dm.L3; % Shin (105.73 mm)
    
    % =========================================================
    % ASSEMBLY BRANCH CONFIGURATION
    % Parallel mechanisms have two assembly modes (e.g., knee bending 
    % forward or backward). These multipliers (1 or -1) select the correct 
    % "bend". If any joint bends the wrong way on the real robot, simply 
    % invert the sign here.
    % =========================================================
    branch_knee = -1; % Controls the knee bend (Mammalian/Dog vs Spider)
    branch_E    = -1;  % Controls if the pull-rod EF goes above or below
    branch_crank = -1; % Controls if the crank AB bends upwards or downwards
    
    % =========================================================
    % STEP 1: Simple IK of the Main Leg (Find thigh and knee G)
    % =========================================================
    % Distance from origin D(0,0) to foot H(x,y)
    d_DH_sq = x^2 + y^2;
    d_DH = sqrt(d_DH_sq);
    
    % Kinematic Safety Lock: Is the point out of reach?
    if d_DH > (DG + GH) || d_DH < abs(DG - GH)
        q = [NaN; NaN];
        return;
    end
    
    % Angles of triangle D-G-H
    alpha = atan2(y, x); % Absolute angle of the D-H vector
    cos_delta = (DG^2 + d_DH_sq - GH^2) / (2 * DG * d_DH);
    delta = acos(cos_delta);
    
    % Absolute angle of the thigh (D-G)
    thigh_angle = alpha + (branch_knee * delta);
    
    % Solution for the FIRST MOTOR (theta2)
    theta2 = thigh_angle - pi/2;
    
    % Coordinates of Knee G
    G_x = DG * cos(thigh_angle);
    G_y = DG * sin(thigh_angle);
    
    % =========================================================
    % STEP 2: Find Point F (Pull-rod coupling at the knee)
    % =========================================================
    % The Shin angle is the line connecting G to H
    shin_angle = atan2(y - G_y, x - G_x);
    
    % Since the FG bar is collinear and opposite to the shin GH 
    % (It stays "behind" the knee), the G-F vector has the shin 
    % angle + 180 degrees (pi radians)
    F_x = G_x + FG * cos(shin_angle + pi);
    F_y = G_y + FG * sin(shin_angle + pi);
    
    % =========================================================
    % STEP 3: Find Point E (Intersection of circles from D and F)
    % =========================================================
    % Distance between D(0,0) and F
    d_DF_sq = F_x^2 + F_y^2;
    d_DF = sqrt(d_DF_sq);
    phi_DF = atan2(F_y, F_x);
    
    % Triangle D-E-F
    cos_gamma = (DE^2 + d_DF_sq - EF^2) / (2 * DE * d_DF);
    if abs(cos_gamma) > 1
        q = [NaN; NaN]; return; % Unreachable point for the internal loop
    end
    gamma = acos(cos_gamma);
    
    % Absolute angle of the DE bar
    phi_DE = phi_DF + (branch_E * gamma);
    
    % =========================================================
    % STEP 4: Find Point C through the rigid triangular plate
    % =========================================================
    % The CDE triangle is rigid, we calculate its constant interior angle
    cos_CDE = (CD^2 + DE^2 - CE^2) / (2 * CD * DE);
    ang_CDE = acos(cos_CDE);
    
    % We subtract the plate's angle from DE to find C
    phi_CD = phi_DE - ang_CDE;
    C_x = CD * cos(phi_CD);
    C_y = CD * sin(phi_CD);
    
    % =========================================================
    % STEP 5: Find motor 'a' (Intersection of circles from C and A)
    % =========================================================
    % Motor A is at (AD, 0)
    % Vector A-C
    AC_x = C_x - AD;
    AC_y = C_y;
    d_AC_sq = AC_x^2 + AC_y^2;
    d_AC = sqrt(d_AC_sq);
    phi_AC = atan2(AC_y, AC_x);
    
    % Triangle A-B-C
    cos_lambda = (AB^2 + d_AC_sq - BC^2) / (2 * AB * d_AC);
    if abs(cos_lambda) > 1
        q = [NaN; NaN]; return; % Mechanical lock of motor A
    end
    lambda = acos(cos_lambda);
    
    % Solution for the SECOND MOTOR (a)
    a = phi_AC + (branch_crank * lambda);
    
    % Output array
    q = [theta2; a];
end