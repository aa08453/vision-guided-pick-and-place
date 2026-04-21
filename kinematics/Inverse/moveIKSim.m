function moveIKSim(points, numPoints)
    [~, config, fig] = moveIK(NaN, points(1).x, points(1).y, points(1).z, NaN, NaN, [0 pi/2 0 0]);
    
    for i = 2:numPoints
        [~, config, fig] = moveIK(NaN, points(i).x, points(i).y, points(i).z, NaN, fig, config);
    end
end