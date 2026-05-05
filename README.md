# CSable Quadruped Control

This is the control codebase for the CSable quadruped robot, structured with industrial MATLAB OOP standards.

## Project Structure

- `config/`: Configuration files and parameters.
- `core/`: Base classes and interfaces.
- `+control/`: Controllers implementation.
- `+hardware/`: Dynamixel hardware wrappers.
- `+kinematics/`: Kinematics and dynamics equations.
- `+sim/`: Simulation environment wrappers.
- `+utils/`: Utility functions.
- `tests/`: Unit tests.

## How to Run

- To run with real hardware: run `main_hardware.m`
- To run the simulation: run `main_simulation.m`
