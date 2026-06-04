classdef DynamixelInterface < handle
    % DYNAMIXELINTERFACE Class to interface with Dynamixel motors.
    % Supports MIXED protocols on the SAME bus:
    %   - X-Series (XM430) on Protocol 2.0  -> leg joints (Thigh, Crank)
    %   - AX-12    on Protocol 1.0          -> Z-axis holding joints
    %
    % Symmetrical mounting (Left/Right legs) handled via Offsets/Directions.

    properties
        PortNum
        ProtocolVersion        % Protocol used by the X-Series joints (2.0)
        ProtocolVersionAX      % Protocol used by the AX-12 joints (1.0)
        MotorIDs               % X-Series leg motor IDs (e.g. [1 2] or [4 3])
        AXMotorIDs             % AX-12 Z-axis motor IDs (e.g. [11 12])
        BaudRate
        DeviceName
        Offsets                % Mounting offsets (radians) for X-Series leg motors
        Directions             % Mounting directions (+1/-1) for X-Series leg motors
        GroupNum               % GroupSyncWrite handle for X-Series Goal Position
        UseSyncWrite           % Flag: true if Sync Write was set up successfully
    end

    properties (Constant)
        % XM430-W350 Control Table (X-Series, Protocol 2.0) 
        ADDR_TORQUE_ENABLE    = 64;   
        ADDR_PROFILE_VELOCITY = 112;  
        ADDR_GOAL_POSITION    = 116;  
        ADDR_PRESENT_POSITION = 132; 

        % AX-12 Control Table (Protocol 1.0) 
        AX_ADDR_CW_COMPLIANCE_MARGIN  = 26;  
        AX_ADDR_CCW_COMPLIANCE_MARGIN = 27;  
        AX_ADDR_CW_COMPLIANCE_SLOPE   = 28; 
        AX_ADDR_CCW_COMPLIANCE_SLOPE  = 29; 
        AX_ADDR_TORQUE_ENABLE  = 24;  
        AX_ADDR_MOVING_SPEED   = 32; 
        AX_ADDR_GOAL_POSITION  = 30; 
        AX_ADDR_PRESENT_POS    = 36; 
        AX_ADDR_TORQUE_LIMIT   = 34;  

        TORQUE_ENABLE  = 1;
        TORQUE_DISABLE = 0;

        % AX-12 specifics: 1024 steps over 300 degrees
        AX_RESOLUTION = 1024;
        AX_RANGE_DEG  = 300;
        AX_CENTER_VAL = 512;                    % 150 deg == center == raw value 512

        LIB_NAME = 'dxl_x64_c';
    end

    methods
        function obj = DynamixelInterface(hw_params)
            % Constructor
            obj.MotorIDs        = hw_params.DXL_IDS;
            obj.DeviceName      = hw_params.DEVICENAME;
            obj.ProtocolVersion = hw_params.PROTOCOL_VERSION;     % 2.0 for X-Series
            obj.BaudRate        = hw_params.BAUDRATE;

            % AX-12 protocol is always 1.0
            if isfield(hw_params, 'PROTOCOL_VERSION_AX')
                obj.ProtocolVersionAX = hw_params.PROTOCOL_VERSION_AX;
            else
                obj.ProtocolVersionAX = 1.0;
            end

            % AX-12 motor IDs (Z-axis holders). Optional.
            if isfield(hw_params, 'AX_IDS')
                obj.AXMotorIDs = hw_params.AX_IDS;
            else
                obj.AXMotorIDs = [];
            end

            % Offsets for X-Series leg motors
            if isfield(hw_params, 'OFFSETS')
                obj.Offsets = hw_params.OFFSETS;
            else
                obj.Offsets = zeros(1, length(obj.MotorIDs));
            end

            % Directions for X-Series leg motors
            if isfield(hw_params, 'DIRECTIONS')
                obj.Directions = hw_params.DIRECTIONS;
            else
                obj.Directions = ones(1, length(obj.MotorIDs));
            end
        end

        function init(obj)
            % Initialize connection and torque for BOTH protocols.

            % Load the Dynamixel SDK library if not already loaded
            if ~libisloaded(obj.LIB_NAME)
                [~, ~] = loadlibrary(obj.LIB_NAME, 'dynamixel_sdk.h', ...
                    'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
            end

            % Initialize Port and the packet handlers
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

            % Enable torque + profile velocity for X-Series leg motors (Proto 2.0)
            safe_speed_val = 0;                     
            for id = obj.MotorIDs
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, ...
                    obj.ADDR_TORQUE_ENABLE, obj.TORQUE_ENABLE);
                write4ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, ...
                    obj.ADDR_PROFILE_VELOCITY, safe_speed_val);
            end
            fprintf('X-Series leg motors initialized and torque enabled.\n');

            ax_speed_val           = 1023;          % moderate move speed to the goal
            ax_torque_limit        = 1023;          % max torque (0..1023). 1023 = full.
            ax_compliance_margin   = 0;             % no dead-zone (0..254 steps)
            ax_compliance_slope    = 32;            % stiff response (0..254, lower = stiffer overall but values below 16 can chatter; 32 is a firm/safe default)
            for id = obj.AXMotorIDs
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersionAX, id, ...
                    obj.AX_ADDR_TORQUE_ENABLE, obj.TORQUE_DISABLE);

                % Stiffness: zero dead-zone, firm slope on both directions
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersionAX, id, ...
                    obj.AX_ADDR_CW_COMPLIANCE_MARGIN,  ax_compliance_margin);
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersionAX, id, ...
                    obj.AX_ADDR_CCW_COMPLIANCE_MARGIN, ax_compliance_margin);
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersionAX, id, ...
                    obj.AX_ADDR_CW_COMPLIANCE_SLOPE,   ax_compliance_slope);
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersionAX, id, ...
                    obj.AX_ADDR_CCW_COMPLIANCE_SLOPE,  ax_compliance_slope);

                % Allow max torque available
                write2ByteTxRx(obj.PortNum, obj.ProtocolVersionAX, id, ...
                    obj.AX_ADDR_TORQUE_LIMIT, ax_torque_limit);

                % Speed at which the motor seeks the goal
                write2ByteTxRx(obj.PortNum, obj.ProtocolVersionAX, id, ...
                    obj.AX_ADDR_MOVING_SPEED, ax_speed_val);

                % Finally, enable torque -> motor now actively holds position
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersionAX, id, ...
                    obj.AX_ADDR_TORQUE_ENABLE, obj.TORQUE_ENABLE);
            end
            if ~isempty(obj.AXMotorIDs)
                fprintf('AX-12 Z-axis motors: stiffened (compliance margin=0, slope=%d, torque=%d) and enabled.\n', ...
                    ax_compliance_slope, ax_torque_limit);
            end

            % Set up GroupSyncWrite for X-Series Goal Position (Proto 2.0).
            obj.UseSyncWrite = false;
            try
                LEN_GOAL_POSITION = 4; 
                obj.GroupNum = groupSyncWrite(obj.PortNum, obj.ProtocolVersion, ...
                    obj.ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
                obj.UseSyncWrite = true;
                fprintf('GroupSyncWrite enabled for X-Series goal positions.\n');
            catch ME
                warning('hardware:DynamixelInterface:GroupSyncWriteUnavailable', ...
                        'GroupSyncWrite not available (%s). Falling back to individual writes.', ME.message);
                obj.UseSyncWrite = false;
            end
        end

        function writeAllLegsSync(obj, all_ids, all_pos_rads)
            % WRITEALLLEGSSYNC Sends goal positions for MULTIPLE X-Series motors
            % in a single Sync Write packet (one serial transaction total).
            %   all_ids      : vector of motor IDs (subset of obj.MotorIDs)
            %   all_pos_rads : matching vector of MODEL radians
            %
            % Falls back to individual TxOnly writes if Sync Write is unavailable.

            if ~obj.UseSyncWrite
                % Fallback: individual non-blocking writes (still faster than TxRx)
                obj.writePosition(all_ids, all_pos_rads);
                return;
            end

            % Build the sync-write parameter list
            for i = 1:length(all_ids)
                id  = all_ids(i);
                idx = find(obj.MotorIDs == id, 1);
                model_rad = all_pos_rads(i);

                if isnan(model_rad)
                    continue;                       % skip unreachable targets, keep last command
                end

                hardware_target_rad = (model_rad * obj.Directions(idx)) + obj.Offsets(idx);
                dxl_val = utils.rad2dxl(hardware_target_rad);

                % Add this motor's 4-byte goal position to the group packet
                groupSyncWriteAddParam(obj.GroupNum, id, dxl_val, 4);
            end

            % Transmit the whole group in ONE packet, then clear for next tick
            groupSyncWriteTxPacket(obj.GroupNum);
            groupSyncWriteClearParam(obj.GroupNum);
        end

        function holdZAxis(obj, degs)
            % HOLDZAXIS Commands the AX-12 motors to fixed angles (degrees).
            % degs may be:
            %   - a scalar  -> same angle applied to every AX-12 motor, or
            %   - a vector  -> one angle per motor, matching obj.AXMotorIDs order.
            % 150 deg is the mechanical center of the AX-12 (raw value 512).
            if isempty(obj.AXMotorIDs)
                return;
            end

            % Expand a scalar to one value per motor
            if isscalar(degs)
                degs = repmat(degs, 1, length(obj.AXMotorIDs));
            end

            if length(degs) ~= length(obj.AXMotorIDs)
                error(['holdZAxis: number of angles (%d) must match number of ' ...
                       'AX-12 motors (%d).'], length(degs), length(obj.AXMotorIDs));
            end

            for i = 1:length(obj.AXMotorIDs)
                id      = obj.AXMotorIDs(i);
                dxl_val = obj.axDeg2Val(degs(i));
                write2ByteTxRx(obj.PortNum, obj.ProtocolVersionAX, id, ...
                    obj.AX_ADDR_GOAL_POSITION, dxl_val);
            end
        end

        function pos_rad = readPosition(obj, motor_id)
            % READPOSITION Reads an X-Series leg motor, returns pure Model Radians.

            dxl_val = double(read4ByteTxRx(obj.PortNum, obj.ProtocolVersion, ...
                motor_id, obj.ADDR_PRESENT_POSITION));

            idx = find(obj.MotorIDs == motor_id, 1);

            raw_hardware_rad = utils.dxl2rad(dxl_val);

            % Reverse transform: Model = (Hardware - Offset) / Direction
            pos_rad = (raw_hardware_rad - obj.Offsets(idx)) * obj.Directions(idx);
        end

        function writePosition(obj, motor_ids, pos_rads)
            % WRITEPOSITION Accepts pure Model Radians for X-Series leg motors,
            % applies direction/offset, sends to motors (Protocol 2.0).

            for i = 1:length(motor_ids)
                id  = motor_ids(i);
                idx = find(obj.MotorIDs == id, 1);

                model_rad = pos_rads(i);

                if isnan(model_rad)
                    continue;
                end

                % Model -> Physical: Hardware = (Model * Direction) + Offset
                hardware_target_rad = (model_rad * obj.Directions(idx)) + obj.Offsets(idx);

                dxl_val = utils.rad2dxl(hardware_target_rad);

                write4ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, ...
                    obj.ADDR_GOAL_POSITION, dxl_val);
            end
        end

        function cleanup(obj)
            % Cleanup the connection safely (both protocols).

            % Disable torque on X-Series leg motors
            for id = obj.MotorIDs
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersion, id, ...
                    obj.ADDR_TORQUE_ENABLE, obj.TORQUE_DISABLE);
            end

            % Disable torque on AX-12 Z-axis motors
            for id = obj.AXMotorIDs
                write1ByteTxRx(obj.PortNum, obj.ProtocolVersionAX, id, ...
                    obj.AX_ADDR_TORQUE_ENABLE, obj.TORQUE_DISABLE);
            end

            closePort(obj.PortNum);
            fprintf('Port closed and motors disabled safely.\n');
        end

        function delete(obj)
            % Destructor - guarantees motors relax if MATLAB ends.
            % Guard against being called before the port was ever opened.
            try
                if ~isempty(obj.PortNum)
                    obj.cleanup();
                end
            catch
                % Silent: nothing safe to do if the port handle is invalid.
            end
        end
    end

    methods (Access = private)
        function val = axDeg2Val(obj, deg)
            % Convert an AX-12 angle in degrees (0..300) to a raw step value (0..1023).
            deg = max(0, min(obj.AX_RANGE_DEG, deg));   
            val = round(deg / obj.AX_RANGE_DEG * (obj.AX_RESOLUTION - 1));
        end
    end
end