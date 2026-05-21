function params = robot_params()
    % ROBOT_PARAMS Establish the robot parameters and characteristics
    % Outputs:
    %   params - Struct with the following elements (Hardware - hw, Dimensions - dm)
    
    % Hardware definitions (XM430-W350-R)
    params.hw.DXL_IDS = [4 3];          % [Thigh Motor ID, Crank Motor ID]
    params.hw.DEVICENAME = 'COM5';      
    params.hw.PROTOCOL_VERSION = 2.0;   % X-Series STRICTLY uses 2.0
    params.hw.BAUDRATE = 1000000;       
    
    % Dimensions definitions (Strictly in millimeters)
    params.dm.L1 = 40.00;
    params.dm.L2 = 100.00; % Thigh
    params.dm.L3 = 105.73; % Shin
end