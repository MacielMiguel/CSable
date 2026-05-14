function params = robot_params()
    % Establish the robot parameters and caracteristhics
    % Outputs:
    %   params - Struct with the following elements (Hardware - hw, Dimentions - dm, Control - ctrl)
    %       hw.DXL_IDS              dm.L1       
    %       hw.DEVICENAME           dm.L2
    %       hw.PROTOCOL_VERSION     dm.L3
    %       hw.BAUDRATE
    %

    % Hardware definitions
    params.hw.DXL_IDS = [2 3];          % Motors IDs
    params.hw.DEVICENAME = "COM5";      
    params.hw.PROTOCOL_VERSION = 1.0;   % Protocol 1.0 or 2.0
    params.hw.BAUDRATE = 1000000;       

    % Dimentions definitions (mm)
    % params.dm.L1 = 40;
    params.dm.L2 = 100;
    params.dm.L3 = 100;
end
