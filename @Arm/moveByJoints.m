function moveByJoints(obj, jointAnglesCommanded)
    prevAngles = obj.jointAngles(1:4);
    obj.lastIKSolutions = [];  % no IK performed — clear ghost arms

    if ~obj.isSimulated
        obj.arb.setpos(4, jointAnglesCommanded(4), obj.speed);
        obj.arb.setpos(3, jointAnglesCommanded(3), obj.speed);
        obj.arb.setpos(2, jointAnglesCommanded(2) + pi/2, obj.speed);
        obj.arb.setpos(1, jointAnglesCommanded(1), obj.speed);
    end

    obj.jointAngles(1:4) = jointAnglesCommanded(1:4);
    obj.ensurePlot();

    if obj.show_trajectory_interpolated
        obj.animateInterpolation(prevAngles, jointAnglesCommanded(1:4));
    else
        obj.updatePlot();
    end
end
