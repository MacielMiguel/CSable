% MAIN_SIMULATION Entry point to run the controller in simulation

% 1. Load configuration
addpath('config');
params = robot_params();

% 2. Initialize Simulator Interface (Mock)
% sim = sim.QuadrupedSimulator();
% sim.init();

% 3. Initialize Robot and Controller
robot = core.Robot(params);
ctrl = control.VirtualModelControl(0.01);

% 4. Control Loop
disp('Starting control loop in simulation...');
for k = 1:100
    % Read state from sim
    % Compute action
    % Send action to sim
end

disp('Simulation finished.');
