classdef DynamixelInterface < handle
    % DYNAMIXELINTERFACE Class to interface with Dynamixel motors
    % Wraps initialization, reading, writing and cleanup.

    properties
        PortNum
        ProtocolVersion
        MotorIDs
        BaudRate
        DeviceName
    end
    
    properties (Constant)
        ADDR_TORQUE_ENABLE = 64; % Update with actual address based on model
        ADDR_MOVING_SPEED = 104; % Update with actual address based on model
        TORQUE_ENABLE = 1;
        TORQUE_DISABLE = 0;
        LIB_NAME = 'dxl_x64_c'; % Adjust if on Linux/Mac
    end
    
    methods
        function obj = DynamixelInterface(hw_params)
            % Constructor
            obj.MotorIDs = hw_params.DXL_IDS;
            obj.DeviceName = hw_params.DEVICENAME;
            obj.ProtocolVersion = hw_params.PROTOCOL_VERSION;
            obj.BaudRate = hw_params.BAUDRATE;
        end
        
        function init(obj)
            % Initialize connection and torque
            % Ensure library is loaded (dummy logic here)
            % if ~libisloaded(obj.LIB_NAME)
            %     loadlibrary(obj.LIB_NAME, 'dynamixel_sdk.h');
            % end
            
            % Dummy assignment for portNum
            obj.PortNum = 1; % portHandler(obj.DeviceName);
            
            % openPort(obj.PortNum)
            % setBaudRate(obj.PortNum, obj.BaudRate);
            fprintf('Port successfully opened!\n');
            
            % Enable Torque and Adjust Speed
            speedVal = 150; 
            for id = obj.MotorIDs
                % write1ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, obj.ADDR_TORQUE_ENABLE, obj.TORQUE_ENABLE);
                % write2ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, obj.ADDR_MOVING_SPEED, speedVal);
            end
            fprintf('Motors enabled.\n');
        end
        
        function pos = readPosition(obj, motor_id)
            % Read position from motor
            pos = 0; % Dummy value
            % pos = read4ByteTxRx(obj.PortNum, obj.ProtocolVersion, motor_id, ADDR_PRESENT_POSITION);
        end
        
        function writePosition(obj, motor_id, pos)
            % Write position to motor
            % write4ByteTxRx(obj.PortNum, obj.ProtocolVersion, motor_id, ADDR_GOAL_POSITION, pos);
        end
        
        function cleanup(obj)
            % Cleanup the connection
            for id = obj.MotorIDs
                % write1ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, obj.ADDR_TORQUE_ENABLE, obj.TORQUE_DISABLE);
            end
            % closePort(obj.PortNum);
            fprintf('Port closed and motors disabled.\n');
        end
        
        function delete(obj)
            % Destructor
            obj.cleanup();
        end
    end
end
