function pos = forward_kinematics(q, params)
    % FORWARD_KINEMATICS Computes the forward kinematics of the leg
    % using a pure analytical geometric approach (no solvers).
    
    % Unpacking state data
    theta2 = q(1); % Angle of the Thigh
    a = q(2);      % Angle of the Crank AB
    
    % =========================================================
    % LINK LENGTHS (From precise CAD/Schematic dimensions in mm)
    % =========================================================
    AD = 50; % Distance between motors (Ground link)
    AB = 20.10; % Crank
    BC = 29.49; % Coupler
    CD = 28.07; % Rocker side 1
    DE = 27.94; % Rocker side 2
    CE = 38.18; % Rocker side 3
    EF = 100.00; % Pull-rod
    FG = 27.27; % Knee crank
    
    % Main leg dimensions (can be overridden by params struct)
    DG = params.dm.L2; % Thigh (100.00 mm)
    GH = params.dm.L3; % Shin (105.73 mm)
    
    % Safety clamp for fixed triangles to avoid floating point errors
    clamp = @(v) max(min(v, 1), -1);
    
    % =========================================================
    % 1. SOLVING THE SUPERIOR LOOP (4-Bar Mechanism A-B-C-D)
    % =========================================================
    % Motor A is at (45.95, 0). D is at (0,0).
    B_x = AD + AB * cos(a);
    B_y = AB * sin(a);
    
    % Distance and angle of the dynamic diagonal D-B
    d_DB_sq = B_x^2 + B_y^2;
    d_DB = sqrt(d_DB_sq);
    phi_DB = atan2(B_y, B_x);
    
    % Law of cosines in Triangle B-C-D to find interior angle at D
    cos_CDB = (CD^2 + d_DB_sq - BC^2) / (2 * CD * d_DB);
    
    % Kinematic safety check: If cos > 1, the linkage has snapped/disconnected
    if abs(cos_CDB) > 1
        pos = [NaN; NaN; NaN];
        return;
    end
    
    ang_CDB = acos(cos_CDB);
    phi_CD = phi_DB + ang_CDB; % Absolute angle of the rocker CD
    
    % =========================================================
    % 2. SOLVING THE RIGID TRIANGLE (C-D-E)
    % =========================================================
    % The interior angle of the rigid triangle CDE is constant
    cos_CDE = (CD^2 + DE^2 - CE^2) / (2 * CD * DE);
    ang_CDE = acos(clamp(cos_CDE)); % Approx 85.9 degrees
    
    % Absolute angle and coordinates of point E
    phi_DE = phi_CD + ang_CDE;
    E_x = DE * cos(phi_DE);
    E_y = DE * sin(phi_DE);
    
    % =========================================================
    % 3. SOLVING THE INFERIOR LOOP (Knee Quadrilateral D-E-F-G)
    % =========================================================
    % Angle of the Thigh (DG) driven directly by motor theta2
    thigh_angle = theta2 + pi/2; 
    
    % Coordinates of point G (The Knee)
    G_x = DG * cos(thigh_angle);
    G_y = DG * sin(thigh_angle);
    
    % Dynamic diagonal E-G to resolve the parallelogram
    d_EG_sq = (E_x - G_x)^2 + (E_y - G_y)^2;
    d_EG = sqrt(d_EG_sq);
    
    % Calculating the relative knee angle (theta3)
    % Angle 1: Inside Triangle D-E-G
    cos_DGE = (DG^2 + d_EG_sq - DE^2) / (2 * DG * d_EG);
    ang_DGE = acos(clamp(cos_DGE));
    
    % Angle 2: Inside Triangle E-F-G
    cos_EGF = (d_EG_sq + FG^2 - EF^2) / (2 * d_EG * FG);
    
    % Second kinematic safety check: Prevents the knee from breaking
    if abs(cos_EGF) > 1
        pos = [NaN; NaN; NaN];
        return;
    end
    ang_EGF = acos(cos_EGF);
    
    % The total relative angle of the shin relative to the thigh
    theta3 = ang_DGE + ang_EGF;
    
    % =========================================================
    % 4. FINAL CARTESIAN MAPPING (End Effector H)
    % =========================================================
    % Absolute angle of the shin in the world frame
    shin_angle = thigh_angle + theta3; 
    
    % Final Cartesian coordinates of the foot
    x = G_x + GH * cos(shin_angle);
    y = G_y + GH * sin(shin_angle);
    z = 0;
    
    pos = [x; y; z];
end