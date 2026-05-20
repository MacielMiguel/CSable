function angles = inverse_kinematics(ref, params)
    x = ref(1);
    z = ref(2);
    L1 = params.L1;
    L2 = params.L2;
    
    D = (x^2 + z^2 - L1^2 - L2^2) / (2 * L1 * L2);
    
    if D > 1 || D < -1
        angles = [NaN; NaN];
        return;
    end
    
    theta2 = -acos(D); 
    theta1 = atan2(z, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
    angles = [theta1; theta2];
end