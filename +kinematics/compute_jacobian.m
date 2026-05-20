function J = compute_jacobian(angles, params)
    theta1 = angles(1);
    theta2 = angles(2);
    L1 = params.L1;
    L2 = params.L2;
    
    J11 = -L1 * sin(theta1) - L2 * sin(theta1 + theta2);
    J12 = -L2 * sin(theta1 + theta2);
    J21 = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    J22 = L2 * cos(theta1 + theta2);
    
    J = [J11, J12; J21, J22];
end