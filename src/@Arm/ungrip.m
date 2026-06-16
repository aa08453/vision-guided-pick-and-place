function ungrip(obj)
    obj.gripCloseness  = 0;
    obj.grippingStatus = false;

    % Drop the cube at the current end-effector position
    if obj.cubeHeld
        [T, ~] = obj.DH();
        obj.cubePos  = T(1:3, 4)';
        obj.cubeHeld = false;
    end

    if ~obj.isSimulated
        obj.arb.setpos(5, 0);
    end

    obj.ensurePlot();
    obj.updatePlot();
end
