function leg_viewer(cfg)
% LEG_VIEWER  Interactive viewer of the full assembled leg (points A..H)
% overlaid on the reachable workspace. Drag the two sliders (theta2 = hip,
% a = crank/knee) and watch the whole linkage redraw in real time.
%
% Usage:
%   >> leg_viewer            % uses sane defaults (crank=+1, AD=41)
%   >> leg_viewer(cfg)       % pass your own cfg struct
%
% Also provides two reusable helpers at the bottom:
%   P = leg_points([theta2;a], cfg)   -> struct with fields A B C D E F G H
%   draw_leg([theta2;a], cfg, ax)     -> static single-pose plot
%
% NOTE: the geometry here is the CORRECTED FK (lower loop by circle
% intersection, shin collinear with FG) and uses branch_crank = +1.

    if nargin < 1 || isempty(cfg)
        cfg.AD            = 41;        % keep equal to your IK!
        cfg.params.dm.L2  = 100.00;   % thigh DG
        cfg.params.dm.L3  = 105.73;   % shin  GH
        cfg.branch_E      = -1;
        cfg.branch_crank  = 1;       % +1 = "elbow up" (matches the sketch)
        cfg.a_offset = deg2rad(90);
    end

    % Motor limits (deg) -- set to your real limits
    TH_MIN = 90;  TH_MAX = 160;
    A_MIN  = 95;  A_MAX  = 175;

    % ---- precompute workspace cloud -------------------------------------
    N = 45;
    tv = linspace(deg2rad(TH_MIN), deg2rad(TH_MAX), N);
    av = linspace(deg2rad(A_MIN),  deg2rad(A_MAX),  N);
    Xc = []; Yc = [];
    for t = tv
        for a = av
            P = leg_points([t; a], cfg);
            if ~any(isnan(P.H))
                Xc(end+1) = P.H(1); %#ok<AGROW>
                Yc(end+1) = P.H(2); %#ok<AGROW>
            end
        end
    end

    % ---- build figure + axes --------------------------------------------
    f  = figure('Name','Leg Viewer','Color','white','Position',[200 150 760 640]);
    ax = axes('Parent',f,'Position',[0.10 0.24 0.86 0.70]);
    scatter(ax, Xc, Yc, 6, [0.80 0.85 0.83], 'filled'); hold(ax,'on');
    plot(ax, 0,0,'ks','MarkerFaceColor','k','MarkerSize',7);
    text(ax, 4,4,'Hip D','FontWeight','bold');
    axis(ax,'equal'); grid(ax,'on'); set(ax,'FontSize',11);
    xlabel(ax,'X (mm)'); ylabel(ax,'Y (mm)');

    % ---- sliders + labels -----------------------------------------------
    uicontrol(f,'Style','text','Units','normalized','Position',[0.10 0.135 0.18 0.03],...
        'String','theta2 (hip)','HorizontalAlignment','left','BackgroundColor','white','FontWeight','bold');
    s_t = uicontrol(f,'Style','slider','Units','normalized','Position',[0.28 0.135 0.55 0.035],...
        'Min',TH_MIN,'Max',TH_MAX,'Value',135,'Callback',@redraw);
    t_t = uicontrol(f,'Style','text','Units','normalized','Position',[0.85 0.135 0.12 0.03],...
        'String','135','BackgroundColor','white');

    uicontrol(f,'Style','text','Units','normalized','Position',[0.10 0.07 0.18 0.03],...
        'String','a (crank/knee)','HorizontalAlignment','left','BackgroundColor','white','FontWeight','bold');
    s_a = uicontrol(f,'Style','slider','Units','normalized','Position',[0.28 0.07 0.55 0.035],...
        'Min',A_MIN,'Max',A_MAX,'Value',135,'Callback',@redraw);
    t_a = uicontrol(f,'Style','text','Units','normalized','Position',[0.85 0.07 0.12 0.03],...
        'String','135','BackgroundColor','white');

    redraw();   % first draw

    % ---- nested redraw ---------------------------------------------------
    function redraw(~,~)
        th = get(s_t,'Value');  a = get(s_a,'Value');
        set(t_t,'String',sprintf('%.0f',th));
        set(t_a,'String',sprintf('%.0f',a));
        delete(findobj(ax,'Tag','leg'));   % wipe previous pose

        P = leg_points([deg2rad(th); deg2rad(a)], cfg);
        if any(isnan(P.H))
            title(ax, sprintf('theta2=%.0f  a=%.0f   ->  UNREACHABLE pose', th, a));
            return;
        end
        draw_leg([deg2rad(th); deg2rad(a)], cfg, ax);   % draws tagged 'leg'
        title(ax, sprintf('theta2=%.0f^o   a=%.0f^o    foot H=(%.1f, %.1f) mm', ...
            th, a, P.H(1), P.H(2)));
    end
end


% =========================================================================
function P = leg_points(q, cfg)
% Returns all joint coordinates A..H for q=[theta2;a] using the corrected FK.
    theta2 = q(1);  a = q(2) + deg2rad(90);
    AD = cfg.AD;
    AB = 20.10; BC = 29.49; CD = 28.07; DE = 27.94; CE = 38.18;
    EF = 100.00; FG = 27.27;
    DG = cfg.params.dm.L2;  GH = cfg.params.dm.L3;
    bE = cfg.branch_E;  bC = cfg.branch_crank;

    nanP = struct('A',[NaN NaN],'B',[NaN NaN],'C',[NaN NaN],'D',[0 0], ...
                  'E',[NaN NaN],'F',[NaN NaN],'G',[NaN NaN],'H',[NaN NaN]);

    A = [AD, 0];  D = [0, 0];
    B = A + AB*[cos(a), sin(a)];

    % --- superior 4-bar -> C ---
    dDB = hypot(B(1),B(2));  phiDB = atan2(B(2),B(1));
    cCDB = (CD^2 + dDB^2 - BC^2)/(2*CD*dDB);
    if abs(cCDB) > 1, P = nanP; return; end
    phiCD = phiDB + bC*acos(cCDB);
    C = CD*[cos(phiCD), sin(phiCD)];

    % --- rigid plate -> E ---
    angCDE = acos(max(min((CD^2+DE^2-CE^2)/(2*CD*DE),1),-1));
    phiDE  = phiCD + angCDE;
    E = DE*[cos(phiDE), sin(phiDE)];

    % --- thigh -> G ---
    thigh = theta2 + pi/2;
    G = DG*[cos(thigh), sin(thigh)];

    % --- lower loop: F = circle(E,EF) ^ circle(G,FG) ---
    d = hypot(E(1)-G(1), E(2)-G(2));
    if d > (EF+FG) || d < abs(EF-FG), P = nanP; return; end
    al = (EF^2 - FG^2 + d^2)/(2*d);
    h  = sqrt(max(EF^2 - al^2, 0));
    u  = (G - E)/d;
    M  = E + al*u;
    F  = [M(1) - bE*h*u(2), M(2) + bE*h*u(1)];

    % --- shin collinear & opposite to FG -> H ---
    s = G - F;  s = s/hypot(s(1),s(2));
    H = G + GH*s;

    P = struct('A',A,'B',B,'C',C,'D',D,'E',E,'F',F,'G',G,'H',H);
end


% =========================================================================
function draw_leg(q, cfg, ax)
% Static draw of the assembled leg on axes ax (creates a figure if omitted).
% All graphics are tagged 'leg' so an interactive caller can delete them.
    if nargin < 3 || isempty(ax)
        figure('Color','white'); ax = axes; axis(ax,'equal'); grid(ax,'on'); hold(ax,'on');
    end
    P = leg_points(q, cfg);
    if any(isnan(P.H)), warning('Unreachable pose.'); return; end

    seg = @(p,qq,c,w) plot(ax,[P.(p)(1) P.(qq)(1)],[P.(p)(2) P.(qq)(2)], ...
                           '-','Color',c,'LineWidth',w,'Tag','leg');

    seg('A','B',[0 0 0],       3);            % crank AB (black)
    seg('B','C',[0 0.6 0],     2);            % coupler BC (green)
    seg('C','D',[0 0 0.8],     2);            % rigid plate (blue)
    seg('D','E',[0 0 0.8],     2);
    seg('C','E',[0 0 0.8],     2);
    patch(ax,'XData',[P.C(1) P.D(1) P.E(1)],'YData',[P.C(2) P.D(2) P.E(2)], ...
          'FaceColor',[0 0 1],'FaceAlpha',0.12,'EdgeColor','none','Tag','leg');
    seg('D','G',[0.3 0.3 0.3], 2);            % thigh DG (grey)
    seg('E','F',[0.8 0 0.8],   2);            % pull-rod EF (magenta)
    seg('F','G',[1 0.55 0],    2);            % knee crank FG (orange)
    seg('G','H',[0.85 0 0],    3);            % shin GH (red)

    names = {'A','B','C','D','E','F','G','H'};
    for i = 1:numel(names)
        p = P.(names{i});
        plot(ax, p(1), p(2), 'ko','MarkerFaceColor','w','MarkerSize',5,'Tag','leg');
        text(ax, p(1)+3, p(2)+3, names{i}, 'FontWeight','bold','Tag','leg');
    end
    plot(ax, P.H(1), P.H(2), 'o','MarkerSize',9,'LineWidth',1.5, ...
         'MarkerEdgeColor',[0.85 0 0],'Tag','leg');   % highlight foot
end