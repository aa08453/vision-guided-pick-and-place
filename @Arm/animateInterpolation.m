function animateInterpolation(obj, startAngles, targetAngles)
    nSteps = 20;
    for k = 1:nSteps
        t = k / nSteps;
        obj.jointAngles(1:4) = (1 - t) * startAngles + t * targetAngles;
        obj.updatePlot();
    end
    obj.jointAngles(1:4) = targetAngles;
end
