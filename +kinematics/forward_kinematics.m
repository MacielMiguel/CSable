function pos = forward_kinematics(q, params)
    % FORWARD_KINEMATICS Computes the forward kinematics of the leg
    % using a pure analytical geometric approach, synchronized with IK branches.
    
    % Unpacking state data
    theta2 = q(1); % Angle of the Thigh
    a = q(2);      % Angle of the Crank AB
    
    % =========================================================
    % LINK LENGTHS 
    % =========================================================
    AD = 41; % Distance between motors
    AB = 20.10; % Crank
    BC = 29.49; % Coupler
    CD = 28.07; % Rocker side 1
    DE = 27.94; % Rocker side 2
    CE = 38.18; % Rocker side 3
    EF = 100.00; % Pull-rod
    FG = 27.27; % Knee crank
    
    DG = params.dm.L2; % Thigh
    GH = params.dm.L3; % Shin
    
    clamp = @(v) max(min(v, 1), -1);
    
    % =========================================================
    % ASSEMBLY BRANCH CONFIGURATION (MUST MATCH IK EXACTLY)
    % =========================================================
    branch_knee  = -1; 
    branch_E     = -1;  
    branch_crank = -1; 
    
    % =========================================================
    % 1. SOLVING THE SUPERIOR LOOP (4-Bar Mechanism A-B-C-D)
    % =========================================================
    B_x = AD + AB * cos(a);
    B_y = AB * sin(a);
    
    d_DB_sq = B_x^2 + B_y^2;
    d_DB = sqrt(d_DB_sq);
    phi_DB = atan2(B_y, B_x);
    
    cos_CDB = (CD^2 + d_DB_sq - BC^2) / (2 * CD * d_DB);
    if abs(cos_CDB) > 1
        pos = [NaN; NaN; NaN]; return;
    end
    ang_CDB = acos(cos_CDB);
    
    % APPLYING CRANK BRANCH
    phi_CD = phi_DB + (branch_crank * ang_CDB); 
    
    % =========================================================
    % 2. SOLVING THE RIGID TRIANGLE (C-D-E)
    % =========================================================
    cos_CDE = (CD^2 + DE^2 - CE^2) / (2 * CD * DE);
    ang_CDE = acos(clamp(cos_CDE)); 
    
    phi_DE = phi_CD + ang_CDE;
    E_x = DE * cos(phi_DE);
    E_y = DE * sin(phi_DE);
    
    % =========================================================
    % 3. SOLVING THE INFERIOR LOOP (Knee Quadrilateral)
    % =========================================================
    thigh_angle = theta2 + pi/2; 
    
    G_x = DG * cos(thigh_angle);
    G_y = DG * sin(thigh_angle);
    
    d_EG_sq = (E_x - G_x)^2 + (E_y - G_y)^2;
    d_EG = sqrt(d_EG_sq);
    
    cos_DGE = (DG^2 + d_EG_sq - DE^2) / (2 * DG * d_EG);
    ang_DGE = acos(clamp(cos_DGE));
    
    cos_EGF = (d_EG_sq + FG^2 - EF^2) / (2 * d_EG * FG);
    if abs(cos_EGF) > 1
        pos = [NaN; NaN; NaN]; return;
    end
    ang_EGF = acos(cos_EGF);
    
    % APPLYING E-BRANCH
    relative_knee_angle = ang_DGE + (branch_E * ang_EGF);
    
    % =========================================================
    % 4. FINAL CARTESIAN MAPPING
    % =========================================================
    % APPLYING KNEE BRANCH
    shin_angle = thigh_angle + (branch_knee * relative_knee_angle); 
    
    x = G_x + GH * cos(shin_angle);
    y = G_y + GH * sin(shin_angle);
    z = 0;
    
    pos = [x; y; z];
end