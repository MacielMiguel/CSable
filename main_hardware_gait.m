% main_hardware_gait.m
% Real-time hardware execution with TWO modes selectable from the terminal:
%   - STANDING : both feet held at X=0, Y=-160
%   - WALKING  : synchronized cycloid gait on LEFT and RIGHT legs
%
% Two AX-12 motors (IDs 11, 12) hold the Z-axis fixed at 150 deg (center).
%
% Controls (real-time keys; keep the small key window focused):
%   SPACE -> toggle between WALKING and STANDING
%   q     -> quit (safe shutdown)

clear; clc; close all;

%% =========================================================
%% 0. GAIT / SYNC CONFIGURATION
%% =========================================================
% How the two legs are phased during walking:
%   'ANTIPHASE' -> legs offset by half a cycle (one swings while the other stances)
%   'INPHASE'   -> both legs do the exact same motion at the same time
GAIT_PHASE_MODE = 'ANTIPHASE';

% AX-12 Z-axis holding angles (degrees), one per motor.
% Order MUST match hw_params.AX_IDS below: [ID 11, ID 12].
Z_HOLD_DEG = [150, 150];   % ID 11 -> 155 deg, ID 12 -> 145 deg

%% 1. Setup Kinematics Parameters & Dimensions
Ts    = 0.01;            % Sample time (100 Hz)
freq  = 1;               % Gait frequency (Hz): 1 cycle per second

% Robot Kinematics Parameters (millimeters)
params = struct();
params.dm.L2 = 100.00;   % Thigh
params.dm.L3 = 105.73;   % Shin

%% 2. Hardware Parameters (single shared port, all motors)
hw_params = struct();
hw_params.DEVICENAME           = 'COM5';   % Check your COM port
hw_params.BAUDRATE             = 1000000;  % Must match BOTH X-Series and AX-12
hw_params.PROTOCOL_VERSION     = 2.0;      % X-Series leg motors
hw_params.PROTOCOL_VERSION_AX  = 1.0;      % AX-12 Z-axis motors

% ---- RIGHT leg (X-Series) ----
RIGHT.DXL_IDS    = [1, 2];          % [Thigh, Crank]
RIGHT.DIRECTIONS = [1, 1];
RIGHT.OFFSETS    = [deg2rad(0), deg2rad(0)];

% ---- LEFT leg (X-Series) ----
% Physical assignment: Thigh = ID 3, Crank = ID 4 (order MUST match RIGHT:
% [Thigh, Crank]). This was previously [4, 3], which swapped the two joints.
% OFFSETS reordered to follow: old [ID4, ID3] -> new [ID3, ID4].
% old: [360 (ID4=Thigh), 324 (ID3=Crank)]  ->  new: [324 (ID3=Thigh? ), 360 ...]
% IMPORTANT: verify these on hardware (see note in chat) before fast motion.
LEFT.DXL_IDS     = [3, 4];          % [Thigh, Crank]
LEFT.DIRECTIONS  = [-1, -1];        % Inverted for symmetry
LEFT.OFFSETS     = [deg2rad(0), deg2rad(-17)];  % [Thigh(ID3), Crank(ID4)]

% ---- AX-12 Z-axis motors ----
hw_params.AX_IDS = [11, 12];        % [Right-Z, Left-Z]

% Combine all leg motors into the single interface.
% Order: [RIGHT thigh, RIGHT crank, LEFT thigh, LEFT crank]
hw_params.DXL_IDS    = [RIGHT.DXL_IDS,    LEFT.DXL_IDS];
hw_params.DIRECTIONS = [RIGHT.DIRECTIONS, LEFT.DIRECTIONS];
hw_params.OFFSETS    = [RIGHT.OFFSETS,    LEFT.OFFSETS];

% Index helpers into the combined arrays
IDS_RIGHT = RIGHT.DXL_IDS;
IDS_LEFT  = LEFT.DXL_IDS;

%% 3. Define the Cycloid Trajectory (one full gait cycle)
% Bounding box: Width = 2*A, Height = B (swing clearance)
Xc = 15;      % Center X
Yc = -160;   % Center Y (also the standing height)
A  = 30;     % Half-width
B  = 30;     % Swing height clearance

T_cycle = 1 / freq;

% Build ONE reference cycle sampled at Ts so we can index it by phase.
t_cycle  = 0:Ts:(T_cycle - Ts);
Ncyc     = length(t_cycle);
cyc_x    = zeros(1, Ncyc);
cyc_y    = zeros(1, Ncyc);

swing_mask  = t_cycle < (T_cycle / 2);
stance_mask = ~swing_mask;

% SWING PHASE (cycloid in the air)
tau_swing = t_cycle(swing_mask) / (T_cycle / 2);
cyc_x(swing_mask) = (Xc - A) + (2*A/(2*pi)) * (2*pi*tau_swing - sin(2*pi*tau_swing));
cyc_y(swing_mask) = (Yc - B) + B * (1 - cos(2*pi*tau_swing));

% STANCE PHASE (linear drag on the ground)
tau_stance = (t_cycle(stance_mask) - (T_cycle/2)) / (T_cycle/2);
cyc_x(stance_mask) = (Xc + A) - (2*A * tau_stance);
cyc_y(stance_mask) = (Yc - B);

% Standing point
STAND_X = 0;
STAND_Y = -180;

% Both legs track the SAME cycloid reference. The left/right difference in
% physical orientation is handled entirely by the motor DIRECTIONS/OFFSETS,
% not by mirroring the trajectory. (An earlier X-mirror band-aid was removed
% once the real cause -- swapped LEFT motor IDs -- was fixed above.)
cyc_x_left  = cyc_x;
cyc_y_left  = cyc_y;
cyc_x_right = cyc_x;
cyc_y_right = cyc_y;

% Phase offset (in samples) between the two legs.
%   'ANTIPHASE' -> half-cycle shift, legs alternate (one swings, one stances)
%   'INPHASE'   -> no shift, both legs do the same motion at the same time
switch upper(GAIT_PHASE_MODE)
    case 'ANTIPHASE'
        phase_shift = round(Ncyc / 2);
    case 'INPHASE'
        phase_shift = 0;
    otherwise
        error('GAIT_PHASE_MODE must be ''ANTIPHASE'' or ''INPHASE''.');
end

%% 3.5 Workspace & Trajectory Pre-Visualization (safety check)
disp('Calculating Workspace for safety verification...');
[~, ~] = kinematics.calc_workspace(params, 60);

fig_workspace = gcf;
set(fig_workspace, 'Name', 'Workspace and Cartesian Path');
title('Workspace and Physical Path (cycloid + standing point)');
hold on;
plot(cyc_x_left,  cyc_y_left,  'b--', 'LineWidth', 2, 'DisplayName', 'LEFT Cycloid');
plot(cyc_x_right, cyc_y_right, 'r:',  'LineWidth', 1.5, 'DisplayName', 'RIGHT Cycloid (same path)');
plot(STAND_X, STAND_Y, 'gp', 'MarkerSize', 12, 'MarkerFaceColor', 'g', ...
    'DisplayName', 'Standing Point');
legend('Location', 'best');
drawnow;

%% 3.6 Verify ENTIRE trajectory (and standing point) is inside the workspace
% Check BOTH legs' trajectories (left = normal, right = X-mirrored) plus the
% standing point, so nothing unreachable can ever be commanded.
disp('Verifying every reference point is reachable...');
check_x = [cyc_x_left, cyc_x_right, STAND_X];
check_y = [cyc_y_left, cyc_y_right, STAND_Y];
for k = 1:length(check_x)
    q_test = kinematics.inverse_kinematics([check_x(k); check_y(k)], params);
    if any(isnan(q_test))
        error(['WORKSPACE ERROR: Reference point (X=%.2f, Y=%.2f) is ' ...
               'unreachable. Adjust Xc, Yc, A, B or the standing point.'], ...
               check_x(k), check_y(k));
    end
end
disp('All reference points verified inside the workspace.');

%% 4. Initialize Subsystems
% 4a. Controllers (one per leg, so each keeps its own warm-start / limiter)
ctrl_right = control.OpenLoopControl(Ts, params);
ctrl_left  = control.OpenLoopControl(Ts, params);

% Warm start both controllers at the standing point
q_stand = kinematics.inverse_kinematics([STAND_X; STAND_Y], params);
ctrl_right.PreviousAction = q_stand;
ctrl_left.PreviousAction  = q_stand;

% 4b. Hardware (single shared interface for all motors)
disp('Initializing hardware connection...');
hw_interface = hardware.DynamixelInterface(hw_params);
hw_interface.init();

% Hold the Z-axis AX-12 motors at center and keep them there.
disp('Holding Z-axis (AX-12) at 150 deg...');
hw_interface.holdZAxis(Z_HOLD_DEG);

% Move both legs to the standing position before anything fast happens.
disp('Moving to standing position. Please stand clear...');
hw_interface.writePosition(IDS_RIGHT, q_stand);
hw_interface.writePosition(IDS_LEFT,  q_stand);
pause(2.0);

%% 5. STATE MACHINE / MAIN LOOP
% Modes: 0 = STANDING, 1 = WALKING, -1 = QUIT
mode = 0;
phase_idx = 0;   % gait phase counter (advances only while walking)

% Create the real-time key-capture window (must stay focused).
kfig = makeKeyWindow();
updateKeyWindow(kfig, mode);

running = true;
while running
    loop_start = tic;

    % --- Real-time, non-blocking key read ---
    % SPACE toggles walk/stand; q quits. drawnow services the key callback
    % without blocking the loop, preserving real-time pacing.
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
        otherwise
            % no key this tick
    end

    if mode == -1
        running = false;
        break;
    end

    % --- Compute references for this tick ---
    if mode == 1
        % WALKING: advance gait phase and sample each leg's own cycloid
        phase_idx = mod(phase_idx + 1, Ncyc);

        idx_right = phase_idx + 1;                           % 1-based
        idx_left  = mod(phase_idx + phase_shift, Ncyc) + 1;  % phase-shifted

        ref_right = [cyc_x_right(idx_right); cyc_y_right(idx_right)];
        ref_left  = [cyc_x_left(idx_left);   cyc_y_left(idx_left)];
    else
        % STANDING: hold both feet at the standing point and reset phase so
        % the next WALK always starts cleanly at the beginning of the cycle.
        phase_idx = 0;
        ref_right = [STAND_X; STAND_Y];
        ref_left  = [STAND_X; STAND_Y];
    end

    % --- Control actions (IK + limiter) per leg ---
    act_right = ctrl_right.computeAction([], ref_right);
    act_left  = ctrl_left.computeAction([],  ref_left);

    % --- Send to hardware: ALL leg motors in ONE Sync Write packet ---
    % Order must match: [RIGHT thigh, RIGHT crank, LEFT thigh, LEFT crank]
    all_ids  = [IDS_RIGHT, IDS_LEFT];
    all_acts = [act_right(:); act_left(:)];
    hw_interface.writeAllLegsSync(all_ids, all_acts);

    % NOTE: holdZAxis is intentionally NOT called here. The AX-12 motors are
    % parked once before the loop and hold position on their own. Re-commanding
    % them every tick added two serial transactions and slowed the loop.

    % --- Real-time pacing ---
    elapsed = toc(loop_start);

    % Diagnostic: print actual loop time. Set DEBUG_TIMING=false to silence.
    DEBUG_TIMING = false;
    if DEBUG_TIMING
        fprintf('loop %.1f ms\n', elapsed*1000);
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
%% Helper functions
%% =========================================================
function f = makeKeyWindow()
    % Hidden-ish figure that captures keystrokes in real time.
    f = figure('Name', 'GAIT KEYS', 'NumberTitle', 'off', ...
               'MenuBar', 'none', 'ToolBar', 'none', ...
               'Color', [0.1 0.1 0.1], 'Position', [50 50 360 140]);
    % Store the last key in the figure's UserData (read by readKey).
    f.UserData = '';
    set(f, 'KeyPressFcn', @(src, ev) setfield2(src, ev.Key));
    uicontrol(f, 'Style', 'text', 'Tag', 'status', ...
        'Units', 'normalized', 'Position', [0 0 1 1], ...
        'FontSize', 13, 'ForegroundColor', 'w', ...
        'BackgroundColor', [0.1 0.1 0.1], ...
        'String', '');
end

function setfield2(src, key)
    % KeyPressFcn callback: stash the pressed key on the figure.
    src.UserData = key;
end

function k = readKey(f)
    % Consume and return the last buffered key (or '' if none).
    if ~isvalid(f)
        k = 'q';   % window was closed -> treat as quit
        return;
    end
    k = f.UserData;
    f.UserData = '';   % consume
end

function updateKeyWindow(f, mode)
    if ~isvalid(f); return; end
    h = findobj(f, 'Tag', 'status');
    if mode == 1
        state = 'WALKING';
        col   = [0.2 0.9 0.3];
    else
        state = 'STANDING';
        col   = [0.9 0.7 0.2];
    end
    set(h, 'ForegroundColor', col, 'String', ...
        sprintf('STATE: %s\n\n[SPACE] toggle   [q] quit\n(keep this window focused)', state));
end