function rad = dxl2rad(dxl_val)
    % DXL2RAD Converts the Dynamixel XM430 position value back to radians.
    
    steps_per_revolution = 4096;
    center_offset = 2048;
    
    % Transform steps back to radians
    rad = (dxl_val - center_offset) * ((2*pi) / steps_per_revolution); 
end
