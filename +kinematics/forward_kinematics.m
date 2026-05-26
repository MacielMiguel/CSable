function pos = forward_kinematics(q, params)
    % FORWARD_KINEMATICS Computes the forward kinematics of the leg
    % using a pure analytical geometric approach, synchronized with IK branches.
    
    % Unpacking state data
    theta2 = q(1); % Angle of the Thigh
    a = q(2);      % Angle of the Crank AB
    a_offset = deg2rad(90); % Offset to stablish the zero position of the crank as vertical (pointing upwards)
    
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
    branch_crank = 1; 
    
    % =========================================================
    % 1. SOLVING THE SUPERIOR LOOP (4-Bar Mechanism A-B-C-D)
    % =========================================================
    a_geo = a + a_offset;          % sua leitura (de +Y) -> ângulo geométrico (de +X)
    B_x = AD + AB * cos(a_geo);
    B_y = AB * sin(a_geo);
    
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
    % 3. SOLVING THE INFERIOR LOOP (Knee Quadrilateral D-E-F-G)
    %    F = intersection of circle(E, EF) and circle(G, FG)
    % =========================================================
    thigh_angle = theta2 + pi/2; 
    G_x = DG * cos(thigh_angle);
    G_y = DG * sin(thigh_angle);
    
    d_EG = hypot(E_x - G_x, E_y - G_y);
    
    % Reachability lock for the lower loop
    if d_EG > (EF + FG) || d_EG < abs(EF - FG)
        pos = [NaN; NaN; NaN]; return;
    end
    
    % Circle-circle intersection (base point along E->G, then perpendicular)
    a_len = (EF^2 - FG^2 + d_EG^2) / (2 * d_EG);
    h_off = sqrt(max(EF^2 - a_len^2, 0));
    ux = (G_x - E_x) / d_EG;  uy = (G_y - E_y) / d_EG;
    Mx = E_x + a_len * ux;    My = E_y + a_len * uy;
    
    % branch_E selects which side F lands on. Use -1 to match the IK exactly.
    F_x = Mx - branch_E * h_off * uy;
    F_y = My + branch_E * h_off * ux;
    
    % =========================================================
    % 4. FINAL CARTESIAN MAPPING
    %    Shin GH is collinear with AND opposite to the knee crank FG
    % =========================================================
    shin_dx = G_x - F_x;
    shin_dy = G_y - F_y;
    nrm = hypot(shin_dx, shin_dy);
    
    x = G_x + GH * shin_dx / nrm;
    y = G_y + GH * shin_dy / nrm;
    z = 0;
    
    pos = [x; y; z];
end