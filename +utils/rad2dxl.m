function dxl_val = rad2dxl(rad)
    % RAD2DXL Converts absolute radians [0, 2*pi] to Dynamixel ticks [0, 4095]
    
    % 1. Normalize angle to strictly [0, 2*pi] range to avoid wrapping
    rad = mod(rad, 2*pi);
    
    steps_per_rev = 4096;
    
    % 2. Pure absolute mapping (No center offset)
    raw_val = round(rad * (steps_per_rev / (2*pi))); 
    
    % 3. Hardware safety clamp
    dxl_val = max(min(raw_val, 4095), 0);
end