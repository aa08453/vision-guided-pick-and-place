function [configs] = findJointAngles(x,y,z, phi)
    d1 = 13.7; a2 = 10.5; a3 = 10.5; a4 = 11;
    theta1.a = atan2(y, x);
    theta1.b = atan2(-y, -x);
    z_bar = z - d1 - a4 * sin(phi);
    
    configs = zeros(4,4);
    i = 1;

    for t1 = [theta1.a, theta1.b]
        x_bar = x/cos(t1) - a4 * cos(phi);
        y_bar = y/sin(t1) - a4 * cos(phi);
        u = (x_bar^2 + z_bar^2 -a2^2 - a3^2) / (2 * a2 * a3);
        denom = (a2^2 + 2 * a2 * a3 * u + a3^2);
        
        for sign = [1,-1]
            t3 = atan2(sign * sqrt(1-u^2), u);
            costh2 = (y_bar * (a2 + a3 * u) + sign * z_bar * (a3 * sqrt(1 - u^2))) / denom;
            sinth2 = (z_bar * (a2 + a3 * u) - sign * y_bar * (a3 * sqrt(1 - u^2))) / denom;
            t2 = atan2(sinth2, costh2);
            t4 = phi - t3 - t2;
            configs(i,:) = [t1, t2, t3, t4];
            i = i+ 1;
        end
    end

end