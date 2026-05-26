function test_workspace()
% TEST_WORKSPACE  Standalone script to compute, validate and visualize the
% reachable workspace of the parallel leg, using the CORRECTED forward
% kinematics (lower loop solved by circle intersection + collinear shin).
%
% Just run:  >> test_workspace
%
% It produces:
%   (1) A console report of the FK<->IK round-trip error (sanity check).
%   (2) Figure 1: the workspace point cloud + boundary.
%   (3) Figure 2: a few IK->FK target checks (targets vs. reached points).

    clc; close all;

    % =====================================================================
    % GLOBAL CONFIG  --  edit these to match your real robot
    % =====================================================================
    cfg.AD = 41;          % <-- distance between motors. MUST be the same
                          %     value you use in your real IK. (You had 41
                          %     in FK and 50 in IK -- pick the real one.)

    cfg.params.dm.L2 = 100.00;   % Thigh DG
    cfg.params.dm.L3 = 105.73;   % Shin  GH

    % Assembly branches (must be identical in FK and IK)
    cfg.branch_knee  = -1;   % knee bend (mammal vs spider)
    cfg.branch_E     = -1;   % lower-loop side (now: which circle-int. side F)
    cfg.branch_crank = 1;   % crank AB up/down

    % Adjusting the value of a to match the geometric angle (sketch is drawn with a=0 along +Y)
    cfg.a_offset = deg2rad(90)

    % Motor sweep limits (radians).  Set these to your REAL motor limits.
    cfg.a_min      = deg2rad(95);    cfg.a_max      = deg2rad(175);
    cfg.theta2_min = deg2rad(90);    cfg.theta2_max = deg2rad(180);

    cfg.num_points = 60;     % grid resolution (60 -> 3600 poses)

    % =====================================================================
    % STEP 0 -- CONSISTENCY CHECK  (does IK invert the corrected FK?)
    % =====================================================================
    fprintf('--- Round-trip validation (FK -> IK -> FK) ---\n');
    maxerr = 0; nok = 0; nfail = 0;
    for th = linspace(cfg.theta2_min, cfg.theta2_max, 15)
        for aa = linspace(cfg.a_min, cfg.a_max, 15)
            p1 = forward_kinematics([th; aa], cfg);
            if any(isnan(p1)), continue; end
            q  = inverse_kinematics(p1, cfg);
            if any(isnan(q)), nfail = nfail + 1; continue; end
            p2 = forward_kinematics(q, cfg);
            if any(isnan(p2)), nfail = nfail + 1; continue; end
            e = norm(p2(1:2) - p1(1:2));
            maxerr = max(maxerr, e); nok = nok + 1;
        end
    end
    fprintf('  checked %d poses, IK failed on %d.\n', nok, nfail);
    fprintf('  max FK->IK->FK position error = %.3e mm\n', maxerr);
    if maxerr < 1e-6
        fprintf('  OK: FK and IK are mutually consistent.\n\n');
    else
        fprintf('  WARNING: large error -> branches/AD mismatch somewhere.\n\n');
    end

    % =====================================================================
    % STEP 1 -- WORKSPACE SWEEP
    % =====================================================================
    a_vec      = linspace(cfg.a_min,      cfg.a_max,      cfg.num_points);
    theta2_vec = linspace(cfg.theta2_min, cfg.theta2_max, cfg.num_points);

    N = cfg.num_points^2;
    X = nan(N,1);  Y = nan(N,1);
    idx = 1;
    for i = 1:numel(theta2_vec)
        for j = 1:numel(a_vec)
            pos = forward_kinematics([theta2_vec(i); a_vec(j)], cfg);
            X(idx) = pos(1);  Y(idx) = pos(2);
            idx = idx + 1;
        end
    end
    valid = ~isnan(X) & ~isnan(Y);
    Xc = X(valid);  Yc = Y(valid);
    fprintf('Workspace: %d of %d poses are reachable.\n', numel(Xc), N);

    % =====================================================================
    % STEP 2 -- PLOT WORKSPACE
    % =====================================================================
    figure('Name','Workspace Analysis','Color','white');
    scatter(Xc, Yc, 15, 'filled', ...
        'MarkerFaceColor', [0.066 0.79 0.627], 'MarkerFaceAlpha', 0.4);
    hold on;
    if numel(Xc) > 3
        k = boundary(Xc, Yc, 0.8);
        plot(Xc(k), Yc(k), 'Color', [0 0.2 0.4], 'LineWidth', 2);
    end
    plot(0,0,'ko','MarkerSize',8,'MarkerFaceColor','k');
    text(5,5,'Hip D (0,0)','FontWeight','bold');
    title('Reachable Workspace (corrected FK)');
    xlabel('X (mm)'); ylabel('Y (mm)');
    axis equal; grid on; set(gca,'FontSize',12);

    % =====================================================================
    % STEP 3 -- VISUAL IK->FK SPOT CHECK
    % A handful of targets inside the cloud: run IK, then FK, and see if
    % the reached point lands on the target (markers should overlap).
    % =====================================================================
    figure('Name','IK->FK spot check','Color','white');
    scatter(Xc, Yc, 8, [0.8 0.8 0.8], 'filled'); hold on;
    % sample some interior targets from the valid cloud
    rng(0);
    nT = min(8, numel(Xc));
    pick = round(linspace(1, numel(Xc), nT));
    for t = 1:nT
        tgt = [Xc(pick(t)); Yc(pick(t)); 0];
        q   = inverse_kinematics(tgt, cfg);
        if any(isnan(q)), continue; end
        rch = forward_kinematics(q, cfg);
        plot(tgt(1), tgt(2), 'o', 'MarkerSize',11, 'LineWidth',1.5, ...
             'MarkerEdgeColor',[0 0.2 0.4]);                 % target (ring)
        plot(rch(1), rch(2), 'x', 'MarkerSize',9, 'LineWidth',2, ...
             'Color',[0.85 0.1 0.1]);                        % reached (X)
    end
    legend({'workspace','target','IK\rightarrowFK reached'}, 'Location','best');
    title('Targets (o) vs. IK\rightarrowFK reached (x) -- should overlap');
    xlabel('X (mm)'); ylabel('Y (mm)');
    axis equal; grid on; set(gca,'FontSize',12);
end


% =========================================================================
%  CORRECTED FORWARD KINEMATICS
% =========================================================================
function pos = forward_kinematics(q, cfg)
    theta2 = q(1);   % thigh
    a      = q(2);   % crank AB

    AD = cfg.AD;
    AB = 20.10;  BC = 29.49;  CD = 28.07;  DE = 27.94;  CE = 38.18;
    EF = 100.00; FG = 27.27;
    DG = cfg.params.dm.L2;   GH = cfg.params.dm.L3;

    branch_E     = cfg.branch_E;
    branch_crank = cfg.branch_crank;
    clamp = @(v) max(min(v,1),-1);

    % --- 1. SUPERIOR LOOP (4-bar A-B-C-D) ---  (unchanged, verified correct)
    B_x = AD + AB*cos(a);   B_y = AB*sin(a);
    d_DB = hypot(B_x, B_y);  phi_DB = atan2(B_y, B_x);
    cos_CDB = (CD^2 + d_DB^2 - BC^2) / (2*CD*d_DB);
    if abs(cos_CDB) > 1, pos = [NaN;NaN;NaN]; return; end
    phi_CD = phi_DB + branch_crank*acos(cos_CDB);

    % --- 2. RIGID TRIANGLE C-D-E ---  (unchanged)
    ang_CDE = acos(clamp((CD^2 + DE^2 - CE^2) / (2*CD*DE)));
    phi_DE  = phi_CD + ang_CDE;
    E_x = DE*cos(phi_DE);   E_y = DE*sin(phi_DE);

    % --- 3. INFERIOR LOOP (CORRECTED): F = circle(E,EF) ^ circle(G,FG) ---
    thigh_angle = theta2 + pi/2;
    G_x = DG*cos(thigh_angle);  G_y = DG*sin(thigh_angle);

    d_EG = hypot(E_x - G_x, E_y - G_y);
    if d_EG > (EF + FG) || d_EG < abs(EF - FG)
        pos = [NaN;NaN;NaN]; return;             % lower loop cannot close
    end
    a_len = (EF^2 - FG^2 + d_EG^2) / (2*d_EG);
    h_off = sqrt(max(EF^2 - a_len^2, 0));
    ux = (G_x - E_x)/d_EG;   uy = (G_y - E_y)/d_EG;
    Mx = E_x + a_len*ux;     My = E_y + a_len*uy;

    % branch_E picks the side of the intersection (use -1 to match the IK)
    F_x = Mx - branch_E*h_off*uy;
    F_y = My + branch_E*h_off*ux;

    % --- 4. FINAL MAPPING: shin GH collinear with & opposite to crank FG ---
    shin_dx = G_x - F_x;   shin_dy = G_y - F_y;
    nrm = hypot(shin_dx, shin_dy);
    x = G_x + GH*shin_dx/nrm;
    y = G_y + GH*shin_dy/nrm;
    pos = [x; y; 0];
end


% =========================================================================
%  INVERSE KINEMATICS  (your version, only AD/params wired to cfg)
% =========================================================================
function q = inverse_kinematics(pos, cfg)
    x = pos(1);  y = pos(2);

    AD = cfg.AD;
    AB = 20.10;  BC = 29.49;  CD = 28.07;  DE = 27.94;  CE = 38.18;
    EF = 100.00; FG = 27.27;
    DG = cfg.params.dm.L2;   GH = cfg.params.dm.L3;

    branch_knee  = cfg.branch_knee;
    branch_E     = cfg.branch_E;
    branch_crank = cfg.branch_crank;

    % STEP 1: 2-link IK of main leg D-G-H
    d_DH_sq = x^2 + y^2;   d_DH = sqrt(d_DH_sq);
    if d_DH > (DG + GH) || d_DH < abs(DG - GH)
        q = [NaN; NaN]; return;
    end
    alpha = atan2(y, x);
    cos_delta = (DG^2 + d_DH_sq - GH^2) / (2*DG*d_DH);
    delta = acos(max(min(cos_delta,1),-1));
    thigh_angle = alpha + branch_knee*delta;
    theta2 = thigh_angle - pi/2;
    G_x = DG*cos(thigh_angle);  G_y = DG*sin(thigh_angle);

    % STEP 2: point F (FG collinear & opposite to shin GH)
    shin_angle = atan2(y - G_y, x - G_x);
    F_x = G_x + FG*cos(shin_angle + pi);
    F_y = G_y + FG*sin(shin_angle + pi);

    % STEP 3: point E (intersection of circles D and F)
    d_DF_sq = F_x^2 + F_y^2;   d_DF = sqrt(d_DF_sq);
    phi_DF = atan2(F_y, F_x);
    cos_gamma = (DE^2 + d_DF_sq - EF^2) / (2*DE*d_DF);
    if abs(cos_gamma) > 1, q = [NaN; NaN]; return; end
    gamma = acos(cos_gamma);
    phi_DE = phi_DF + branch_E*gamma;

    % STEP 4: point C through rigid plate
    ang_CDE = acos((CD^2 + DE^2 - CE^2) / (2*CD*DE));
    phi_CD = phi_DE - ang_CDE;
    C_x = CD*cos(phi_CD);  C_y = CD*sin(phi_CD);

    % STEP 5: motor a
    AC_x = C_x - AD;  AC_y = C_y;
    d_AC_sq = AC_x^2 + AC_y^2;  d_AC = sqrt(d_AC_sq);
    phi_AC = atan2(AC_y, AC_x);
    cos_lambda = (AB^2 + d_AC_sq - BC^2) / (2*AB*d_AC);
    if abs(cos_lambda) > 1, q = [NaN; NaN]; return; end
    a = phi_AC + branch_crank*acos(cos_lambda);

    q = [theta2; a];
end