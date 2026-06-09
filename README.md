# CSable — Quadruped Robot Control System

A real-time MATLAB control system for a quadruped robot with a **parallel-link leg mechanism**, driven by Dynamixel XM430-W350-R servo motors. The codebase follows industrial MATLAB OOP standards with namespace isolation (`+package` convention), centralized configuration, and hardware safety constraints.

---

## Table of Contents

1. [Requirements](#requirements)
2. [Installation](#installation)
3. [Project Structure](#project-structure)
4. [Kinematics](#kinematics)
5. [Control Modules](#control-modules)
6. [Hardware Interface](#hardware-interface)
7. [Running the Robot](#running-the-robot)
8. [Tests & Visualization](#tests--visualization)
9. [Configuration Reference](#configuration-reference)
10. [Safety Guidelines](#safety-guidelines)

---

## Requirements

### Software

| Software | Minimum Version | Purpose |
|---|---|---|
| MATLAB | R2023b | Main runtime environment |
| Dynamixel SDK (MATLAB) | ≥ 3.7 | Serial communication with motors (Protocol 2.0 and 1.0) |
| Control System Toolbox | Any | Recommended for analysis and plotting |

The Dynamixel SDK must be downloaded separately from the [ROBOTIS official repository](https://github.com/ROBOTIS-GIT/DynamixelSDK) and added to the MATLAB path:

```matlab
addpath(genpath('C:/DynamixelSDK/matlab'));
savepath;
```

### Hardware

| Component | Qty | Specification |
|---|---|---|
| Dynamixel XM430-W350-R | 8 (min.) | Protocol 2.0 — 1 Mbaud — 12 V — 4.1 N·m stall torque |
| Dynamixel AX-12A | 4 | Protocol 1.0 — lateral Z-axis stabilizer |
| ROBOTIS U2D2 | 1 | USB ↔ TTL/RS-485 interface |
| 12 V DC power supply | 1 | ≥ 5 A recommended |
| Windows / Linux PC | 1 | USB port available |

### Motor IDs (default)

| Leg | Thigh ID | Crank ID | AX-12 ID |
|---|---|---|---|
| Front-Right (FR) | 1 | 2 | 12 |
| Front-Left (FL) | 3 | 4 | 11 |
| Rear-Right (RR) | 7 | 8 | 14 |
| Rear-Left (RL) | 5 | 6 | 13 |

> FL and RL legs are mounted mirrored — direction correction is handled automatically via `DIRECTIONS` and `OFFSETS` in `config/robot_params.m`.

---

## Installation

1. **Clone the repository** into a local directory.

2. **Install the Dynamixel SDK** (MATLAB bindings) and add it to the MATLAB path as shown above.

3. **Open MATLAB in the project root** — the directory that contains the `+core`, `+control`, `+hardware`, `+kinematics`, and `+utils` folders. MATLAB must be opened at this level for the `+package` namespaces to resolve correctly.

4. **Configure the serial port** in `config/robot_params.m`:

```matlab
params.PORT_NAME = 'COM5';    % Adjust to your system (e.g. '/dev/ttyUSB0' on Linux)
params.BAUD_RATE  = 1000000;
```

5. **Verify motor IDs** using the Dynamixel Wizard 2.0 tool, and update `robot_params.m` if needed.

---

## Project Structure

```
CSable/
├── config/
│   └── robot_params.m          # Central parameter repository (hardware IDs, geometry, COM port)
│
├── +core/
│   ├── AbstractController.m    # Abstract base class — enforces computeAction() interface
│   └── Robot.m                 # Robot state container (joint positions q and velocities dq)
│
├── +control/
│   ├── OpenLoopControl.m       # Open-loop IK-based trajectory tracking with velocity clamping
│   ├── VirtualModelControl.m   # Impedance control via virtual springs and dampers (τ = Jᵀ·F)
│   └── ModelPredictiveControl.m# MPC skeleton — Q/R weighting, horizon config (not yet implemented)
│
├── +hardware/
│   ├── DynamixelInterface.m    # Position control — Protocol 2.0 (XM430) + Protocol 1.0 (AX-12)
│   └── DynamixelInterfaceVMC.m # Torque/current control variant for VMC
│
├── +kinematics/
│   ├── forward_kinematics.m    # Joint angles → foot Cartesian position
│   ├── inverse_kinematics.m    # Desired foot position → motor angles
│   ├── compute_jacobian.m      # Numerical Jacobian via central finite differences
│   └── calc_workspace.m        # Reachable workspace sweep and visualization
│
├── +utils/
│   ├── rad2dxl.m               # Radians [0, 2π] → Dynamixel steps [0, 4095]
│   └── dxl2rad.m               # Dynamixel steps [0, 4095] → Radians [0, 2π]
│
├── tests/
│   ├── leg_viewer.m            # Interactive 2D linkage GUI with sliders
│   ├── test_workspace.m        # FK→IK→FK round-trip validation + workspace sweep
│   └── main_cycloid_fennec.m   # Biological gait playback from CSV data
│
├── main_hardware_standard_gait.m  # Full quadruped trot gait
├── main_hardware_cycloid.m        # Single-leg cycloid trajectory test
├── main_hardware_ellipse.m        # Single-leg ellipse trajectory test
└── main_hardware_slow_gait.m      # Quadruped walk gait (slower, more stable)
```

---

## Kinematics

Each leg uses a **two-DOF parallel-link mechanism** composed of two closed kinematic loops.

### Mechanism Layout

```
Superior loop (4-bar):  A ── B ── C ── D
                             ↑           ↑
                           crank      hip joint (origin)
Rigid triangle:         C ── D ── E   (rigid plate)
Inferior loop (knee):   D ── E ── F ── G
                                        ↑
                                    knee joint
Shin:                   G ── H    (foot = effector)
```

### Link Lengths

| Segment | Length (mm) | Description |
|---|---|---|
| AD | 41.00 | Motor separation (fixed) |
| AB | 20.10 | Crank |
| BC | 29.49 | Upper coupler |
| CD | 28.07 | Rigid triangle side |
| DE | 27.94 | Rigid triangle side |
| CE | 38.18 | Rigid triangle diagonal |
| EF | 100.00 | Pull-rod |
| FG | 27.27 | Knee crank |
| DG | 100.00 | Thigh (L2) |
| GH | 105.73 | Shin (L3) |

### Coordinate System

- **Origin D**: hip joint
- **X-axis**: forward/backward
- **Y-axis**: vertical (negative toward ground)
- **Nominal standing height**: Y ≈ −150 mm

### Assembly Branch Configuration

```matlab
branch_knee  = -1   % Mammalian (dog-like) knee bend
branch_E     = -1   % Lower loop circle intersection side
branch_crank = +1   % Crank bends upward
```

> The IK and FK must use the **same branch configuration**. Mismatches cause silent round-trip errors — always verify with `tests/test_workspace.m`.

### API

```matlab
cfg = robot_params();

% Forward kinematics: joint angles → foot position
pos = forward_kinematics([theta2; a], cfg);   % returns [X; Y] in mm

% Inverse kinematics: foot position → joint angles
q = inverse_kinematics([X; Y], cfg);          % returns [theta2; a] in rad
                                               % returns [NaN; NaN] if unreachable

% Numerical Jacobian (2×2)
J = compute_jacobian([theta2; a], cfg);

% Workspace analysis and plot
[X, Y] = calc_workspace(cfg);
```

---

## Control Modules

All controllers inherit from `core.AbstractController` and must implement `computeAction(ref)`.

### OpenLoopControl

IK-based position control with velocity rate limiting.

```matlab
ctrl = control.OpenLoopControl(robot_params());
action = ctrl.computeAction([X; Y]);   % returns [theta2; a] in rad
```

| Parameter | Default | Description |
|---|---|---|
| `Ts` | 0.01 s | Sample time (100 Hz) |
| `MAX_VEL` | 240 °/s | Angular velocity limit per step |
| `PreviousAction` | `[NaN; NaN]` | Warm-start — holds last valid command on singularity |

### VirtualModelControl

Impedance control using virtual springs and dampers. Converts Cartesian force to joint torques via the Jacobian transpose: **τ = Jᵀ · F**, where **F = Kp·e + Kd·ė**.

| Parameter | Value | Unit |
|---|---|---|
| `Kp_x` | 500 | N/m |
| `Kp_z` | 300 | N/m |
| `Kd` | 10 | N·s/m |
| `τ_max` | ±3.0 | N·m (motor limit: 4.1 N·m) |

Requires `DynamixelInterfaceVMC` (current-control mode).

### ModelPredictiveControl

Placeholder with Q/R weighting matrices and configurable prediction horizon. Currently returns zero — reserved for future implementation.

---

## Hardware Interface

### DynamixelInterface (position control)

Supports **Protocol 2.0** (XM430) and **Protocol 1.0** (AX-12) on the same RS-485 bus. Uses `GroupSyncWrite` to broadcast all leg positions in a single serial packet, reducing latency.

```matlab
hw = hardware.DynamixelInterface(params);
hw.init();

hw.writePosition(motor_id, angle_rad);   % send position command
q = hw.readPosition(motor_id);           % read current position

hw.writeAllLegsSync(angles_matrix);      % broadcast all 8 motors at once
hw.holdZAxis();                          % lock AX-12 lateral motors
hw.cleanup();                            % disable torque and close port
```

#### Control Table Addresses

| Register | XM430 address | AX-12 address |
|---|---|---|
| TORQUE_ENABLE | 64 | 24 |
| GOAL_POSITION | 116 | 30 |
| PRESENT_POSITION | 132 | 36 |
| PROFILE_VELOCITY | 112 | — |

### DynamixelInterfaceVMC (torque control)

Switches motors to **current control mode (mode 0)**. Converts torque in N·m to raw current units (208.85 raw/N·m).

### Unit Conversion

```matlab
dxl_val   = utils.rad2dxl(angle_rad);   % [0, 2π] → [0, 4095]
angle_rad = utils.dxl2rad(dxl_val);     % [0, 4095] → [0, 2π]
```

Resolution: 4096 steps/revolution → **0.088° per step**.

---

## Running the Robot

All main scripts follow this execution pattern:

```
1. Load parameters (robot_params)
2. Initialize hardware interfaces (DynamixelInterface)
3. Validate full trajectory against reachable workspace (calc_workspace)
4. Smooth stand-up ramp (3 seconds, smoothstep interpolation)
5. Real-time control loop (tic/toc timing, pause to enforce Ts)
6. Safe cleanup in try/catch (hw.cleanup on any exception)
```

### Standard Trot Gait — `main_hardware_standard_gait.m`

Full quadruped diagonal trot. All four legs controlled simultaneously.

| Parameter | Value |
|---|---|
| Gait frequency | 2.0 Hz |
| Trajectory center | Xc = 9 mm, Yc = −150 mm |
| Step amplitude | A = 40 mm (X), B = 20 mm (lift) |
| Phase FR / FL / RR / RL | 0.0 / 0.5 / 0.5 / 0.0 |

**Keyboard controls during execution:**
- `SPACE` — toggle between walking and standing still
- `q` — quit and trigger safe cleanup

### Cycloid Trajectory — `main_hardware_cycloid.m`

Single-leg test using a cycloidal foot path. Logs reference vs. actual positions and plots tracking errors after execution.

```matlab
LEG  = 'RIGHT';   % or 'LEFT'
Xc = 5;  Yc = -160;   % trajectory center (mm)
A  = 50; B  = 20;     % amplitude (mm)
freq = 1;             % Hz
```

### Ellipse Trajectory — `main_hardware_ellipse.m`

Single-leg test using an elliptical foot path. Useful for comparing tracking smoothness against the cycloid.

```matlab
Xc = 40;  Yc = -145;
A  = 20;  B  = 15;
freq = 0.5;
```

### Slow Walk Gait — `main_hardware_slow_gait.m`

Quadruped walk gait with staggered phase offsets — more stable at low speed.

| Parameter | Slow gait | Standard gait |
|---|---|---|
| Frequency | 2.2 Hz | 2.0 Hz |
| Phase FL | 0.75 | 0.50 |
| Phase RL | 0.25 | 0.00 |
| Step amplitude A | 30 mm | 40 mm |

---

## Tests & Visualization

### Interactive Leg Viewer — `tests/leg_viewer.m`

GUI with two sliders to explore leg kinematics in real time. Displays all joint positions A–H, workspace point cloud, and current foot position.

```matlab
cd tests
leg_viewer
```

### Workspace Validation — `tests/test_workspace.m`

Four-step validation procedure:
1. FK → IK → FK round-trip consistency check (reports max error)
2. 60×60 joint-space sweep to collect all reachable points
3. Visualization (point cloud + convex boundary)
4. Spot-check IK → FK for 8 random targets

```matlab
cd tests
test_workspace
```

> Run this before connecting hardware whenever kinematics parameters are changed.

### Biological Gait Playback — `tests/main_cycloid_fennec.m`

Replays a fennec fox gait from `fennec_foot_points.csv`. Pixel coordinates are normalized to the robot workspace. Two legs run in opposition of phase (T/2 offset). The user must visually confirm the trajectory overlay before the hardware loop starts.

---

## Configuration Reference

All robot parameters are centralized in `config/robot_params.m`. Never duplicate these values elsewhere — always call `cfg = robot_params()` and pass the struct as an argument.

| Parameter | Default | Unit | Description |
|---|---|---|---|
| `PORT_NAME` | `'COM5'` | — | Serial port of the U2D2 interface |
| `BAUD_RATE` | 1 000 000 | bps | 1 Mbaud serial baud rate |
| `IDS_THIGH` | `[1, 3, 7, 5]` | — | Thigh motor IDs (FR, FL, RR, RL) |
| `IDS_CRANK` | `[2, 4, 8, 6]` | — | Crank motor IDs |
| `IDS_AX12` | `[12, 11, 14, 13]` | — | AX-12 lateral motor IDs |
| `DIRECTIONS` | `[1, -1, 1, -1]` | — | Mirror correction for left-side legs |
| `Ts` | 0.01 | s | Sample time (100 Hz loop) |
| `L1` | 40 | mm | Crank link length |
| `L2` | 100 | mm | Thigh length DG |
| `L3` | 105.73 | mm | Shin length GH |

---

## Safety Guidelines

- **Never start a main script without physically checking** that the robot is in a safe starting position (legs at mid-travel, clear space around the robot).
- **Workspace pre-validation** runs automatically before every hardware loop — it will abort if any trajectory point is out of reach.
- **Singularity guard:** IK returns `[NaN; NaN]` for unreachable points — all controllers hold the last valid command instead of commanding a zero or invalid position.
- **Velocity limiting:** a 240°/s rate filter is applied every control step to prevent joint damage from instantaneous angle jumps.
- **Torque clamping (VMC):** hard limit at ±3.0 N·m — below the 4.1 N·m stall torque to protect gearboxes.
- **Smooth stand-up:** every script ramps from a tucked position to standing over 3 seconds using smoothstep interpolation.
- **Global try/catch:** any runtime exception triggers `hw.cleanup()` immediately, disabling torque on all motors.
- **SyncWrite:** all 8 motor positions are sent in a single packet to prevent inter-leg desynchronization from cumulative serial latency.

> In case of doubt, cut the power supply before opening MATLAB.
