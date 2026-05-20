function pos = forward_kinematics(angles, params)
    theta1 = angles(1);
    theta2 = angles(2);
    L1 = params.L1;
    L2 = params.L2;
    
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    z = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
    pos = [x; z];
end