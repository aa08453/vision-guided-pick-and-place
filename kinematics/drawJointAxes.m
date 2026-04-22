function drawJointAxes(robot, config, axisLength)
    if nargin < 3
        axisLength = 2;
    end
    
    for i = 1:robot.NumBodies
        bodyName = robot.Bodies{i}.Name;
        tform = getTransform(robot, config, bodyName);
        origin = tform(1:3, 4)';
        
        xAxis = tform(1:3, 1)';
        yAxis = tform(1:3, 2)';
        zAxis = tform(1:3, 3)';
        
        quiver3(origin(1), origin(2), origin(3), ...
                xAxis(1), xAxis(2), xAxis(3), ...
                axisLength, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        quiver3(origin(1), origin(2), origin(3), ...
                yAxis(1), yAxis(2), yAxis(3), ...
                axisLength, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        quiver3(origin(1), origin(2), origin(3), ...
                zAxis(1), zAxis(2), zAxis(3), ...
                axisLength, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        
        text(origin(1), origin(2), origin(3) + 0.5, ...
             sprintf('J%d', i), 'FontSize', 8, 'Color', 'k', 'FontWeight', 'bold');
    end
end