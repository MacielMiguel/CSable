# CSable Quadruped Control System - Documentation

Welcome to the documentation for the **CSable** quadruped robot control codebase. This guide explains the underlying architecture, how the various modules connect, and provides a walkthrough of the core mechanics—designed specifically for someone who is completely new to this system.

---

## 1. Architectural Overview

This system is built using Object-Oriented Programming (OOP) in MATLAB to mimic industry-standard robotics software. To ensure that the code is easy to expand and debug, responsibilities are split into highly specific folders (known as "Namespaces" in MATLAB, denoted by the `+` prefix).

### Folder Structure & Roles

* **`config/`**: Contains scripts like `robot_params.m`. This is the single source of truth for the robot's physical dimensions (e.g., link lengths) and hardware configurations (e.g., motor IDs, COM ports). If you ever change a motor or replace a 3D-printed leg, you only need to update the numbers here.
* **`core/`**: Houses base classes like `AbstractController` and `Robot`. These define the "blueprint" of the system. For instance, `AbstractController` enforces a rule that every controller MUST have a `computeAction` function.
* **`+hardware/`**: Manages the physical actuators. The `DynamixelInterface` class abstracts away the ugly C/C++ library calls into simple commands like `.writePosition()` and `.init()`.
* **`+kinematics/`**: The math hub. It contains `forward_kinematics` (calculating foot position given motor angles) and `inverse_kinematics` (calculating motor angles needed to reach a specific foot position).
* **`+control/`**: Contains the "brain" algorithms. Currently holds `OpenLoopControl` (moves the leg blindly without sensors), but is built to accommodate `VirtualModelControl` and `ModelPredictiveControl` in the future.
* **`+utils/`**: Small helper functions. Crucially, it holds `rad2dxl` and `dxl2rad` which translate standard mathematical radians into the arbitrary `0-4095` step scale that the Dynamixel XM430 motors understand.

---

## 2. How the Control Loop Works

The core of the robotic system is executed inside the `main_hardware.m` script. The script follows a standard robotics pattern: **Initialize $\rightarrow$ Plan $\rightarrow$ Execute $\rightarrow$ Cleanup**.

### Initialization
Before anything moves, the system gathers the parameters, initializes the communication with the hardware (opening the USB port and sending torque-enable signals to the motors), and instantiates the chosen controller (e.g., `OpenLoopControl`).

### The Control Loop (The "Heartbeat")
The robot operates in a continuous loop governed by a sample time, $T_s$ (e.g., $0.01$ seconds, or $100 \text{ Hz}$). Inside this loop, a trajectory is generated and executed:

1. **Planning (Interpolation)**: We don't command the leg to jump instantly from Point A to Point B, as this would draw immense current and damage the hardware. Instead, we generate intermediate "waypoints". The line `current_ref = pointA + (pointB - pointA) * percentage` smoothly glides the target over time.
2. **Inverse Kinematics**: We ask the controller to `computeAction()`. The controller takes the X/Z Cartesian coordinate we just generated and mathematically figures out the exact joint angles ($\theta_1$ and $\theta_2$) required.
3. **Conversion**: The controller outputs standard radians. We pass these through our `+utils` to convert them into integers (e.g., 2048, 2500) that the Dynamixel firmware expects.
4. **Actuation**: We dispatch the integer positions to the hardware interface.
5. **Pacing**: We use `pause(Ts)` to ensure the loop runs exactly at the intended real-world speed, preventing the computer from sending commands faster than the motors can process them.

### Safety (Try / Catch)
Robotics involves physical hardware that can break. The control loop is wrapped in a `try/catch` block. If any error occurs—such as mathematical singularity (trying to reach too far) or a disconnected wire—the MATLAB execution instantly halts, jumps to the `catch` block, and forcefully runs `dxl.cleanup()`. This immediately cuts the torque to the motors, rendering the leg limp and safe.
