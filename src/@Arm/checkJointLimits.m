function withinLimits = checkJointLimits(obj, jointAngles)
    withinLimits = true;
    for i = 1:length(jointAngles)
        wrappedAngle = abs(wrapToPi(jointAngles(i)));
        if (mod(wrappedAngle, pi) >= deg2rad(150))
            withinLimits = false;
            if obj.show_debug_output
                warning('Joint %d out of limits: %.4f rad (limit %.4f rad)', ...
                        i, jointAngles(i), deg2rad(150));
            end
            break;
        end
    end
end
