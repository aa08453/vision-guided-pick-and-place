function [withinLimits] = checkJointLimits(jointAngles)
    withinLimits = true;
    for i = 1:length(jointAngles)
        if (wrapToPi(abs(jointAngles(i))) >= deg2rad(150))
            withinLimits = false;
            warning("Not within joint limits");
            fprintf("The %d-th joint is out of bounds with: %f, limit is %f", i, jointAngles(i), deg2rad(150))
            break;
        end
    end
end