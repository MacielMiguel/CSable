% main_hardware_gait_quad.m
% Real-time hardware execution for a QUADRUPED robot with TWO modes:
%   - STANDING : all four feet held at X=0, Y=-180
%   - WALKING  : configurable gait (default: diagonal trot)
%
% Leg layout (facing forward):
%   Front-Right (FR): Thigh=ID1, Crank=ID2  |  AX-12 Z: ID 12
%   Front-Left  (FL): Thigh=ID3, Crank=ID4  |  AX-12 Z: ID 11
%   Rear-Right  (RR): Thigh=ID7, Crank=ID8  |  AX-12 Z: ID 14
%   Rear-Left   (RL): Thigh=ID5, Crank=ID6  |  AX-12 Z: ID 13
%
% Controls (keep the small key window focused):
%   SPACE -> toggle WALKING / STANDING
%   q     -> quit (safe shutdown)

clear; clc; close all;

%% =========================================================
%% 0. GAIT / SYNC CONFIGURATION
%% =========================================================
% Individual phase offsets per leg (fraction of cycle, 0.0 to 1.0).
% FR is the reference (0.0). Edit the others to change the gait pattern.
%
%   Gait presets:
%   -------------------------------------------------------
%   Diagonal trot  : FL=0.50  RR=0.50  RL=0.00   (default)
%   Walk (4-beat)  : FL=0.25  RR=0.75  RL=0.50
%   Bound          : FL=0.00  RR=0.50  RL=0.50
%   Pronk (all)    : FL=0.00  RR=0.00  RL=0.00
%   -------------------------------------------------------
PHASE_FR = 0.00;   % Front-Right  (reference)
PHASE_FL = 0.75;   % Front-Left
PHASE_RR = 0.50;   % Rear-Right
PHASE_RL = 0.25;   % Rear-Left

% AX-12 Z-axis holding angles (degrees), one per motor.
% Order MUST match hw_params.AX_IDS below: [ID 11, ID 12, ID 13, ID 14].
Z_HOLD_DEG = [152, 151, 150, 148];   % [FL-Z, FR-Z, RL-Z, RR-Z]

%% 1. Setup Kinematics Parameters & Dimensions
Ts   = 0.01;   % Sample time (100 Hz)
freq = 2.2;      % Gait frequency (Hz)

% Robot Kinematics Parameters (millimeters) — identical for all four legs
params = struct();
params.dm.L2 = 100.00;   % Thigh
params.dm.L3 = 105.73;   % Shin

%% 2. Hardware Parameters (single shared port, all motors)
hw_params = struct();
hw_params.DEVICENAME          = 'COM5';    % <-- check your COM port
hw_params.BAUDRATE            = 1000000;
hw_params.PROTOCOL_VERSION    = 2.0;       % X-Series leg motors
hw_params.PROTOCOL_VERSION_AX = 1.0;       % AX-12 Z-axis motors

% ---- FRONT-RIGHT leg (X-Series) ----
FR.DXL_IDS    = [1, 2];          % [Thigh, Crank]
FR.DIRECTIONS = [1,  1];
FR.OFFSETS    = [deg2rad(0), deg2rad(0)];

% ---- FRONT-LEFT leg (X-Series) ----
FL.DXL_IDS    = [3, 4];          % [Thigh, Crank]
FL.DIRECTIONS = [-1, -1];        % Inverted for symmetry
FL.OFFSETS    = [deg2rad(0), deg2rad(-17)];

% ---- REAR-RIGHT leg (X-Series) ----
% Same mechanical symmetry as Front-Right.
RR.DXL_IDS    = [7, 8];          % [Thigh, Crank]
RR.DIRECTIONS = [1,  1];
RR.OFFSETS    = [deg2rad(0), deg2rad(0)];

% ---- REAR-LEFT leg (X-Series) ----
% Same mechanical symmetry as Front-Left.
RL.DXL_IDS    = [5, 6];          % [Thigh, Crank]
RL.DIRECTIONS = [-1, -1];
RL.OFFSETS    = [deg2rad(0), deg2rad(0)];

% ---- AX-12 Z-axis motors (all four) ----
% Order matches Z_HOLD_DEG above: [FL, FR, RL, RR]
hw_params.AX_IDS = [11, 12, 13, 14];

% Combine ALL leg motors into the single interface.
% Order: [FR thigh, FR crank, FL thigh, FL crank,
%         RR thigh, RR crank, RL thigh, RL crank]
hw_params.DXL_IDS    = [FR.DXL_IDS,    FL.DXL_IDS,    RR.DXL_IDS,    RL.DXL_IDS];
hw_params.DIRECTIONS = [FR.DIRECTIONS, FL.DIRECTIONS, RR.DIRECTIONS, RL.DIRECTIONS];
hw_params.OFFSETS    = [FR.OFFSETS,    FL.OFFSETS,    RR.OFFSETS,    RL.OFFSETS];

% Index helpers
IDS_FR = FR.DXL_IDS;
IDS_FL = FL.DXL_IDS;
IDS_RR = RR.DXL_IDS;
IDS_RL = RL.DXL_IDS;

%% 3. Define the Cycloid Trajectory (one full gait cycle)
Xc = 25;     % Center X
Yc = -160;   % Center Y (standing height)
A  = 30;     % Half-width
B  = 15;     % Swing height clearance

T_cycle = 1 / freq;
t_cycle = 0:Ts:(T_cycle - Ts);
Ncyc    = length(t_cycle);
cyc_x   = zeros(1, Ncyc);
cyc_y   = zeros(1, Ncyc);

swing_mask  = t_cycle < (T_cycle / 2);
stance_mask = ~swing_mask;

% SWING PHASE
tau_swing = t_cycle(swing_mask) / (T_cycle / 2);
cyc_x(swing_mask) = (Xc - A) + (2*A/(2*pi)) * (2*pi*tau_swing - sin(2*pi*tau_swing));
cyc_y(swing_mask) = (Yc - B) + B * (1 - cos(2*pi*tau_swing));

% STANCE PHASE
tau_stance = (t_cycle(stance_mask) - (T_cycle/2)) / (T_cycle/2);
cyc_x(stance_mask) = (Xc + A) - (2*A * tau_stance);
cyc_y(stance_mask) = (Yc - B);

% All four legs use the same cycloid shape.
% Left/right physical symmetry is handled by DIRECTIONS/OFFSETS, not trajectory.
% Front/rear physical symmetry is assumed identical (same kinematics).

% Standing point
STAND_X =    0;
STAND_Y = -180;

%% =========================================================
%% PHASE OFFSETS (converted to sample shifts)
%% =========================================================
% Each leg is indexed independently using its own offset, so any gait
% pattern can be selected just by editing PHASE_* at the top.
%   - Diagonal trot keeps one diagonal pair in swing while the other
%     is in stance, giving two-point support at all times.
shift_fr = round(Ncyc * PHASE_FR);
shift_fl = round(Ncyc * PHASE_FL);
shift_rr = round(Ncyc * PHASE_RR);
shift_rl = round(Ncyc * PHASE_RL);

%% 3.5 Workspace & Trajectory Pre-Visualization (safety check)
disp('Calculating workspace for safety verification...');
[~, ~] = kinematics.calc_workspace(params, 60);

fig_workspace = gcf;
set(fig_workspace, 'Name', 'Workspace and Cartesian Path');
title('Workspace and Cycloid Path');
hold on;
plot(cyc_x,  cyc_y, 'b-',  'LineWidth', 2,   'DisplayName', 'Cycloid (all legs, same shape)');
plot(STAND_X, STAND_Y, 'gp', 'MarkerSize', 12, 'MarkerFaceColor', 'g', ...
    'DisplayName', 'Standing Point');
legend('Location', 'best');
drawnow;

%% 3.6 Verify ENTIRE trajectory and standing point inside workspace
disp('Verifying every reference point is reachable...');
check_x = [cyc_x, STAND_X];
check_y = [cyc_y, STAND_Y];
for k = 1:length(check_x)
    q_test = kinematics.inverse_kinematics([check_x(k); check_y(k)], params);
    if any(isnan(q_test))
        error(['WORKSPACE ERROR: Point (X=%.2f, Y=%.2f) is unreachable. ' ...
               'Adjust Xc, Yc, A, B or standing point.'], check_x(k), check_y(k));
    end
end
disp('All reference points verified inside the workspace.');

%% 4. Initialize Subsystems
% One controller per leg (each keeps its own warm-start / limiter state)
ctrl_fr = control.OpenLoopControl(Ts, params);
ctrl_fl = control.OpenLoopControl(Ts, params);
ctrl_rr = control.OpenLoopControl(Ts, params);
ctrl_rl = control.OpenLoopControl(Ts, params);

% Warm-start all controllers at standing position
q_stand = kinematics.inverse_kinematics([STAND_X; STAND_Y], params);
ctrl_fr.PreviousAction = q_stand;
ctrl_fl.PreviousAction = q_stand;
ctrl_rr.PreviousAction = q_stand;
ctrl_rl.PreviousAction = q_stand;

% Single shared hardware interface for all motors
disp('Initializing hardware connection...');
hw_interface = hardware.DynamixelInterface(hw_params);
hw_interface.init();

% Hold all four Z-axis AX-12 motors
disp('Holding Z-axis (AX-12) for all four legs...');
hw_interface.holdZAxis(Z_HOLD_DEG);

%% =========================================================
%% 4.5 SMOOTH STANDUP — gradual move from a fixed start pose
%% =========================================================
% Interpolates from a FIXED, safe starting pose (a tucked/lower foot
% position) to the standing IK angles using a smooth-step curve
% (ease-in/ease-out, zero velocity at both ends). This avoids reading
% from the hardware entirely, and replaces the old abrupt writePosition
% + pause(2.0) so the robot rises gently on the first run.
%
% START_X / START_Y define where the standup BEGINS (in the same Cartesian
% frame as the standing point). The default is a tucked pose slightly
% higher (less extended) than standing. Adjust if it doesn't match how the
% robot physically sits when powered on.
START_X = 0;
START_Y = -160;   % tucked / lifted foot — closer to the body than standing

% Tune STANDUP_DURATION to control how fast the robot rises.
STANDUP_DURATION = 3.0;   % seconds
STANDUP_STEPS    = round(STANDUP_DURATION / Ts);

% Fixed starting joint angles (same for all four legs, via IK).
q_start = kinematics.inverse_kinematics([START_X; START_Y], params);
if any(isnan(q_start))
    error('SMOOTH STANDUP: start pose (X=%.1f, Y=%.1f) is unreachable.', ...
           START_X, START_Y);
end

fprintf('Smooth standup over %.1f s. Please stand clear...\n', STANDUP_DURATION);

for step = 1:STANDUP_STEPS
    t_standup = tic;

    alpha   = step / STANDUP_STEPS;          % linear 0 -> 1
    alpha_s = alpha^2 * (3 - 2*alpha);       % smooth-step 3t^2 - 2t^3

    % All four legs share the same start->stand ramp.
    q_ramp = q_start + alpha_s * (q_stand - q_start);
    q_fr = q_ramp;
    q_fl = q_ramp;
    q_rr = q_ramp;
    q_rl = q_ramp;

    all_ids  = [IDS_FR,    IDS_FL,    IDS_RR,    IDS_RL];
    all_acts = [q_fr(:); q_fl(:); q_rr(:); q_rl(:)];
    hw_interface.writeAllLegsSync(all_ids, all_acts);

    elapsed = toc(t_standup);
    if Ts > elapsed
        pause(Ts - elapsed);
    end
end

disp('Standup complete. Robot is in standing position.');
pause(0.5);

%% 5. STATE MACHINE / MAIN LOOP
mode      = 0;    % 0 = STANDING, 1 = WALKING, -1 = QUIT
phase_idx = 0;    % gait phase counter (advances only while walking)

kfig = makeKeyWindow();
updateKeyWindow(kfig, mode);

running = true;
while running
    loop_start = tic;

    % --- Non-blocking key read ---
    drawnow limitrate;
    cmd = readKey(kfig);
    switch cmd
        case 'space'
            if mode == 1
                mode = 0;
                disp('>> STANDING');
            else
                mode = 1;
                disp('>> WALKING');
            end
            updateKeyWindow(kfig, mode);
        case 'q'
            disp('>> QUIT requested.');
            mode = -1;
    end

    if mode == -1
        running = false;
        break;
    end

    % --- Compute references for this tick ---
    if mode == 1
        % WALKING: advance phase and index each leg with its own offset
        phase_idx = mod(phase_idx + 1, Ncyc);

        idx_fr = mod(phase_idx + shift_fr, Ncyc) + 1;
        idx_fl = mod(phase_idx + shift_fl, Ncyc) + 1;
        idx_rr = mod(phase_idx + shift_rr, Ncyc) + 1;
        idx_rl = mod(phase_idx + shift_rl, Ncyc) + 1;

        ref_fr = [cyc_x(idx_fr); cyc_y(idx_fr)];
        ref_fl = [cyc_x(idx_fl); cyc_y(idx_fl)];
        ref_rr = [cyc_x(idx_rr); cyc_y(idx_rr)];
        ref_rl = [cyc_x(idx_rl); cyc_y(idx_rl)];
    else
        % STANDING: reset phase for clean walk restart
        phase_idx = 0;
        ref_fr = [STAND_X; STAND_Y];
        ref_fl = [STAND_X; STAND_Y];
        ref_rr = [STAND_X; STAND_Y];
        ref_rl = [STAND_X; STAND_Y];
    end

    % --- IK + limiter per leg ---
    act_fr = ctrl_fr.computeAction([], ref_fr);
    act_fl = ctrl_fl.computeAction([], ref_fl);
    act_rr = ctrl_rr.computeAction([], ref_rr);
    act_rl = ctrl_rl.computeAction([], ref_rl);

    % --- Single GroupSyncWrite for ALL eight leg motors ---
    % Order MUST match hw_params.DXL_IDS:
    %   [FR thigh, FR crank, FL thigh, FL crank,
    %    RR thigh, RR crank, RL thigh, RL crank]
    all_ids  = [IDS_FR,       IDS_FL,       IDS_RR,       IDS_RL];
    all_acts = [act_fr(:); act_fl(:); act_rr(:); act_rl(:)];
    hw_interface.writeAllLegsSync(all_ids, all_acts);

    % NOTE: AX-12 motors are parked once before the loop and hold on their own.

    % --- Real-time pacing ---
    elapsed = toc(loop_start);

    DEBUG_TIMING = false;
    if DEBUG_TIMING
        fprintf('loop %.1f ms\n', elapsed * 1000);
    end

    if elapsed < Ts
        pause(Ts - elapsed);
    end
end

%% 6. Safe Cleanup
disp('Shutting down. Cleaning up hardware...');
hw_interface.cleanup();
if isvalid(kfig); close(kfig); end
disp('Done.');

%% =========================================================
%% Helper functions (identical to bipedal version)
%% =========================================================
function f = makeKeyWindow()
    f = figure('Name', 'GAIT KEYS', 'NumberTitle', 'off', ...
               'MenuBar', 'none', 'ToolBar', 'none', ...
               'Color', [0.1 0.1 0.1], 'Position', [50 50 360 140]);
    f.UserData = '';
    set(f, 'KeyPressFcn', @(src, ev) setfield2(src, ev.Key));
    uicontrol(f, 'Style', 'text', 'Tag', 'status', ...
        'Units', 'normalized', 'Position', [0 0 1 1], ...
        'FontSize', 13, 'ForegroundColor', 'w', ...
        'BackgroundColor', [0.1 0.1 0.1], 'String', '');
end

function setfield2(src, key)
    src.UserData = key;
end

function k = readKey(f)
    if ~isvalid(f)
        k = 'q';
        return;
    end
    k = f.UserData;
    f.UserData = '';
end

function updateKeyWindow(f, mode)
    if ~isvalid(f); return; end
    h = findobj(f, 'Tag', 'status');
    if mode == 1
        state = 'WALKING';  col = [0.2 0.9 0.3];
    else
        state = 'STANDING'; col = [0.9 0.7 0.2];
    end
    set(h, 'ForegroundColor', col, 'String', ...
        sprintf('STATE: %s\n\n[SPACE] toggle   [q] quit\n(keep this window focused)', state));
end