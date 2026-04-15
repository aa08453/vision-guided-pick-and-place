function [configs] = findJointAngles(x,y,z, phi)
    d1 = 13.7; a2 = 10.5; a3 = 10.5; a4 = 11;
   
    theta1pos = atan2(y, x);
    theta1neg= atan2(-y, -x);
    
    i = 1;

    for theta1 = [theta1pos, theta1neg]
        r = sqrt(x^2 + y^2);
        s = z - d1;

        r_bar = r - a4 * cos(phi);
        s_bar = s - a4 * sin(phi);

        
        theta3pos = acos((r_bar^2 + s_bar^2 - a2^2 - a3^2)/(2 * a2 * a3));
        for theta3 = [theta3pos, -theta3pos] 
            theta2 = atan2(s_bar, r_bar) - atan2(a3 * sin(theta3), a2 + a3*cos(theta3));
            theta3mod = theta3;
            theta4 = 0;
            if (theta1 == theta1neg)
                theta2 = wrapToPi(pi - theta2);
                theta3mod = wrapToPi(-theta3);
                theta4 = wrapToPi(pi - phi - theta3mod - theta2);
            elseif (theta1 == theta1pos)
                theta4 = phi - theta3mod - theta2;
            end
           

            configs(i,:) = wrapToPi([theta1, theta2, theta3mod, theta4]);
            i = i+ 1;
        end
    end

end