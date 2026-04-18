function moveIKReal(arb, points, numPoints)
q = getCurrentPose(arb);
[~, config, ~] = moveIK(arb, points(1).x, points(1).y, points(1).z, NaN, NaN, q(1:4));
    
    for i = 2:numPoints
        [~, config, ~] = moveIK(arb, points(i).x, points(i).y, points(i).z, NaN, NaN, config);
    end
end