function solutions = findJointAngles(obj, x, y, z, phi)
    d1 = obj.d(1);
    a2 = obj.a(2);
    a3 = obj.a(3);
    a4 = obj.a(4);



    theta1_candidates = [atan2(y, x), atan2(y, x)+pi];
    solutions = [];

    fprintf('\n=== IK Debug for (%.2f, %.2f, %.2f, phi=%.4f) ===\n', x, y, z, phi);
    fprintf('theta1 candidates: %.4f, %.4f\n', theta1_candidates(1), theta1_candidates(2));

    for t1_idx = 1:2
        theta1 = theta1_candidates(t1_idx);
        r = sqrt(x^2 + y^2);
        s = z - d1;
        r_bar = r - a4 * cos(phi);
        s_bar = s - a4 * sin(phi);
        cos_theta3_arg = (r_bar^2 + s_bar^2 - a2^2 - a3^2) / (2 * a2 * a3);

        fprintf('\n-- t1_idx=%d, theta1=%.4f --\n', t1_idx, wrapToPi(theta1_candidates(t1_idx)));
        fprintf('  r=%.4f, s=%.4f\n', r, s);
        fprintf('  r_bar=%.4f, s_bar=%.4f\n', r_bar, s_bar);
        fprintf('  cos_theta3_arg=%.4f\n', cos_theta3_arg);

        if abs(cos_theta3_arg) > 1
            fprintf('  Position Unreachable\n');
            continue;
        end

        theta3_pos = acos(cos_theta3_arg);
        for theta3_raw = [theta3_pos, -theta3_pos]
            theta2 = atan2(s_bar, r_bar) - atan2(a3 * sin(theta3_raw), a2 + a3 * cos(theta3_raw));
            if t1_idx == 2
                theta2 = (pi - theta2);
                theta3 = -theta3_raw;
                theta4 = (pi - phi - theta3 - theta2);
            else
                theta3 = theta3_raw;
                theta4 = (phi - theta3 - theta2);
            end

            row = wrapToPi([theta1, theta2, theta3, theta4]);

            if any(~isfinite(row))
                fprintf('  SKIPPED: unreachable\n');
                continue;
            end

            fprintf('  theta3=%.4f => [th1=%.4f, th2=%.4f, th3=%.4f, th4=%.4f]\n', ...
                theta3, row(1), row(2), row(3), row(4));
            solutions = [solutions; row];
        end
    end

    if isempty(solutions)
        warning('Arm.findJointAngles: no valid IK solutions for (%.2f, %.2f, %.2f)', x, y, z);
    end
end
