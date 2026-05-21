classdef DynamixelInterface < handle
    % DYNAMIXELINTERFACE Class to interface with Dynamixel XM-Series motors
    % Wraps initialization, reading, writing, conversion, and cleanup.

    properties
        PortNum
        ProtocolVersion
        MotorIDs
        BaudRate
        DeviceName
        Offsets
    end
    
    properties (Constant)
        % XM430-W350 Control Table Addresses (X-Series)
        ADDR_TORQUE_ENABLE    = 64;  % 1 Byte
        ADDR_PROFILE_VELOCITY = 112; % 4 Bytes (Controls movement speed)
        ADDR_GOAL_POSITION    = 116; % 4 Bytes
        ADDR_PRESENT_POSITION = 132; % 4 Bytes
        
        TORQUE_ENABLE  = 1;
        TORQUE_DISABLE = 0;
        
        % Adjust library name based on OS: 
        % Windows: 'dxl_x64_c' | Linux: 'libdxl_x64_c' | Mac: 'libdxl_mac_c'
        LIB_NAME = 'dxl_x64_c'; 
    end
    
    methods
        function obj = DynamixelInterface(hw_params)
            % Constructor
            obj.MotorIDs = hw_params.DXL_IDS;
            obj.DeviceName = hw_params.DEVICENAME;
            obj.ProtocolVersion = hw_params.PROTOCOL_VERSION;
            obj.BaudRate = hw_params.BAUDRATE;

            % Initialize Offsets (Default to 0 if not provided)
            if isfield(hw_params, 'OFFSETS')
                obj.Offsets = hw_params.OFFSETS;
            else
                obj.Offsets = zeros(1, length(obj.MotorIDs));
            end
        end
        
        function init(obj)
            % Initialize connection and torque
            
            % 1. Load the Dynamixel SDK library if not already loaded
            if ~libisloaded(obj.LIB_NAME)
                [notfound, warnings] = loadlibrary(obj.LIB_NAME, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
            end
            
            % 2. Initialize Port and set BaudRate
            obj.PortNum = portHandler(obj.DeviceName);
            packetHandler();
            
            if openPort(obj.PortNum)
                fprintf('Succeeded to open the port!\n');
            else
                error('Failed to open the port! Check permissions and device name.');
            end
            
            if setBaudRate(obj.PortNum, obj.BaudRate)
                fprintf('Succeeded to change the baudrate!\n');
            else
                error('Failed to change the baudrate!');
            end
            
            % 3. Enable Torque and set a safe Profile Velocity
            % Profile velocity of 100 roughly equals ~22.9 rev/min. 
            % Adjust this depending on how fast you want the physical leg to snap to targets.
            safe_speed_val = 100; 
            
            for id = obj.MotorIDs
                % Enable Torque (1 Byte)
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, obj.ADDR_TORQUE_ENABLE, obj.TORQUE_ENABLE);
                
                % Set Profile Velocity (4 Bytes in X-Series)
                write4ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, obj.ADDR_PROFILE_VELOCITY, safe_speed_val);
            end
            fprintf('Motors initialized and torque enabled.\n');
        end
        
        function pos_rad = readPosition(obj, motor_id)
            % READPOSITION Reads hardware steps, applies offset, and returns pure Model Radians
            
            % Read 4-byte position from hardware and FORCE cast to double
            dxl_val = double(read4ByteTxRx(obj.PortNum, obj.ProtocolVersion, motor_id, obj.ADDR_PRESENT_POSITION));
            
            % Find which index this motor corresponds to
            idx = find(obj.MotorIDs == motor_id, 1);
            
            % Convert hardware steps to radians, then SUBTRACT the hardware offset
            raw_hardware_rad = utils.dxl2rad(dxl_val);
            pos_rad = raw_hardware_rad - obj.Offsets(idx);
        end
        
        function writePosition(obj, motor_ids, pos_rads)
            % WRITEPOSITION Accepts pure Model Radians, applies offset, and sends to motors.
            
            for i = 1:length(motor_ids)
                id = motor_ids(i);
                idx = find(obj.MotorIDs == id, 1);
                
                % The mathematical target angle
                model_rad = pos_rads(i);
                
                % Safety Check
                if isnan(model_rad)
                    continue; 
                end
                
                % ADD the hardware offset to match the physical mounting
                hardware_target_rad = model_rad + obj.Offsets(idx);
                
                % Convert Radians to hardware steps using your utility
                dxl_val = utils.rad2dxl(hardware_target_rad);
                
                % Write the 4-byte goal position to the hardware
                write4ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, obj.ADDR_GOAL_POSITION, dxl_val);
            end
        end
        
        function cleanup(obj)
            % Cleanup the connection safely
            for id = obj.MotorIDs
                % Disable torque before closing
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, obj.ADDR_TORQUE_ENABLE, obj.TORQUE_DISABLE);
            end
            
            % Close the port
            closePort(obj.PortNum);
            fprintf('Port closed and motors disabled safely.\n');
            
            % Optional: Unload library to free memory
            % unloadlibrary(obj.LIB_NAME);
        end
        
        function delete(obj)
            % Destructor - guarantees motors relax if MATLAB crashes or script ends
            obj.cleanup();
        end
    end
end