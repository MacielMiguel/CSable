function rad = dxl2rad(dxl_val)
    % DXL2RAD Converts Dynamixel ticks [0, 4095] to absolute radians [0, 2*pi]
    
    % Cast to double immediately to prevent MATLAB uint32 math corruption
    dxl_val = double(dxl_val);
    
    steps_per_rev = 4096;
    
    % Pure absolute mapping
    rad = dxl_val * ((2*pi) / steps_per_rev); 
end