function dxl_val = rad2dxl(rad)
    % RAD2DXL Converts radians to the Dynamixel XM430 position value.
    % The X-series motors have a resolution of 4096 steps.
    
    steps_per_revolution = 4096;
    center_offset = 2048;
    
    % Convert radians to steps and round to nearest integer
    dxl_val = round(rad * (steps_per_revolution / (2*pi)) + center_offset); 
end
