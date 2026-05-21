classdef DynamixelInterfaceVMC < handle
    % DYNAMIXELINTERFACEVMC Hardware interface strictly for Torque/Current control.
    % Designed for Virtual Model Control (VMC) of Dynamixel XM-Series motors.

    properties
        PortNum
        ProtocolVersion
        MotorIDs
        BaudRate
        DeviceName
    end
    
    properties (Constant)
        % XM430-W350 Control Table Addresses (X-Series)
        ADDR_OPERATING_MODE   = 11;  % 1 Byte (0 = Current Control, 3 = Position Control)
        ADDR_TORQUE_ENABLE    = 64;  % 1 Byte
        ADDR_GOAL_CURRENT     = 126; % 2 Bytes (Controls output torque)
        ADDR_PRESENT_POSITION = 132; % 4 Bytes (Read current state)
        
        TORQUE_ENABLE  = 1;
        TORQUE_DISABLE = 0;
        CURRENT_MODE   = 0;
        
        % Adjust library name based on OS (Windows: 'dxl_x64_c')
        LIB_NAME = 'dxl_x64_c'; 
    end
    
    methods
        function obj = DynamixelInterfaceVMC(hw_params)
            % Constructor
            obj.MotorIDs = hw_params.DXL_IDS;
            obj.DeviceName = hw_params.DEVICENAME;
            obj.ProtocolVersion = hw_params.PROTOCOL_VERSION;
            obj.BaudRate = hw_params.BAUDRATE;
        end
        
        function init(obj)
            % Initialize connection, change operating mode, and enable torque
            
            if ~libisloaded(obj.LIB_NAME)
                [~, ~] = loadlibrary(obj.LIB_NAME, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
            end
            
            obj.PortNum = portHandler(obj.DeviceName);
            packetHandler();
            
            if openPort(obj.PortNum)
                fprintf('Succeeded to open the port for VMC!\n');
            else
                error('Failed to open the port! Check permissions and device name.');
            end
            
            if setBaudRate(obj.PortNum, obj.BaudRate)
                fprintf('Succeeded to change the baudrate!\n');
            else
                error('Failed to change the baudrate!');
            end
            
            fprintf('Configuring Dynamixels to Current (Torque) Control Mode...\n');
            for id = obj.MotorIDs
                % 1. Turn OFF torque (Required to change operating mode)
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, obj.ADDR_TORQUE_ENABLE, obj.TORQUE_DISABLE);
                
                % 2. Set Operating Mode to Current Control (0)
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, obj.ADDR_OPERATING_MODE, obj.CURRENT_MODE);
                
                % 3. Turn ON torque
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, obj.ADDR_TORQUE_ENABLE, obj.TORQUE_ENABLE);
            end
            fprintf('Motors initialized and ready in Torque Mode.\n');
        end
        
        function pos_rad = readPosition(obj, motor_id)
            % READPOSITION We still need to read positions in VMC to calculate
            % the Jacobian, Cartesian positions, and velocities.
            
            dxl_val = read4ByteTxRx(obj.PortNum, obj.ProtocolVersion, motor_id, obj.ADDR_PRESENT_POSITION);
            pos_rad = utils.dxl2rad(dxl_val);
        end
        
        function writeTorque(obj, motor_ids, torques_Nm)
            % WRITETORQUE Converts Newton-meters (Nm) to raw hardware current 
            % units and sends them to the motors.
            
            % Conversion factor for XM430-W350:
            % 1 raw unit = 2.69 mA
            % Torque Constant = ~1.78 Nm/A (or 0.00178 Nm/mA)
            % 1 raw unit = 2.69 * 0.00178 = 0.004788 Nm
            % Therefore: Raw Unit = Torque_Nm * 208.85
            Nm_to_raw = 208.85;
            
            for i = 1:length(motor_ids)
                id = motor_ids(i);
                tau = torques_Nm(i);
                
                % Safety Check: Do not send NaNs to the hardware
                if isnan(tau)
                    continue; 
                end
                
                % Convert to raw signed 2-byte integer current value
                raw_current = round(tau * Nm_to_raw);
                
                % Write the 2-byte goal current to the hardware
                write2ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, obj.ADDR_GOAL_CURRENT, raw_current);
            end
        end
        
        function cleanup(obj)
            % Disable torque before closing
            for id = obj.MotorIDs
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, obj.ADDR_TORQUE_ENABLE, obj.TORQUE_DISABLE);
            end
            
            closePort(obj.PortNum);
            fprintf('VMC Port closed and motors relaxed safely.\n');
        end
        
        function delete(obj)
            obj.cleanup();
        end
    end
end