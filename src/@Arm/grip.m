function grip(obj)
    obj.gripCloseness  = 1;
    obj.grippingStatus = true;

    % Attach the cube if the end-effector is within reach
    if ~isempty(obj.cubePos) && ~obj.cubeHeld
        [T, ~] = obj.DH();
        ee_pos = T(1:3, 4)';
        if norm(ee_pos - obj.cubePos) < obj.gripperOpenWidth + 2
            obj.cubeHeld = true;
        end
    end

    if ~obj.isSimulated
        obj.arb.setpos(5,1.5);
    end

    obj.ensurePlot();
    obj.updatePlot();
end
