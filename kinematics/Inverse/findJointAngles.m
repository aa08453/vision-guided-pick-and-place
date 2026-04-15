function [configs] = findJointAngles(x, y, z, phi)
    d1 = 13.7; a2 = 10.5; a3 = 10.5; a4 = 11;

    theta1_candidates = [atan2(y, x), atan2(-y, -x)];
    configs = [];

    for t1_idx = 1:2
        theta1 = theta1_candidates(t1_idx);

        r = sqrt(x^2 + y^2);
        s = z - d1;
        r_bar = r - a4 * cos(phi);
        s_bar = s - a4 * sin(phi);

        cos_theta3_arg = (r_bar^2 + s_bar^2 - a2^2 - a3^2) / (2 * a2 * a3);

        % Guard: skip if position is unreachable
        if abs(cos_theta3_arg) > 1
            continue;
        end

        theta3_pos = acos(cos_theta3_arg);

        for theta3_raw = [theta3_pos, -theta3_pos]
            theta2 = atan2(s_bar, r_bar) - atan2(a3 * sin(theta3_raw), a2 + a3 * cos(theta3_raw));

            if t1_idx == 2  % negative theta1 (back-facing)
                theta2   = wrapToPi(pi - theta2);
                theta3   = wrapToPi(-theta3_raw);
                theta4   = wrapToPi(pi - phi - theta3 - theta2);
            else             % positive theta1 (front-facing)
                theta3   = theta3_raw;
                theta4   = phi - theta3 - theta2;
            end

            row = wrapToPi([theta1, theta2, theta3, theta4]);

            % Skip rows with NaN/Inf
            if any(~isfinite(row))
                continue;
            end

            configs = [configs; row];
        end
    end

    if isempty(configs)
        warning('findJointAngles: no valid IK solutions for (%.2f, %.2f, %.2f)', x, y, z);
    end
end