function success = moveIKReal(arb, points, angles, grip)
numPoints = length(points);
numAngles = size(angles,1);
success = true;


q = getCurrentPose(arb);
if (numAngles~=0)
    angle = angles(1);
else 
    angle = NaN;
end

[validSolution, config, ~] = moveIK(arb, points(1).x, points(1).y, points(1).z, angle, NaN, q(1:4));

if (~isempty(grip))
    pause(2);
    arb.setpos(5, grip(1),100)
    pause(2);
end

if isempty(validSolution)    
    success = false;
end

pause(2);

for i = 2:numPoints
    if (numAngles < numPoints)
        angle = angles(i);
    else 
        angle = NaN;
    end

    [validSolution, config, ~] = moveIK(arb, points(i).x, points(i).y, points(i).z, angle, NaN, config);
    if isempty(validSolution)    
        success = false;
    end
    if (length(grip) <= numPoints)
        pause(2);
        arb.setpos(5, grip(i),100)
        pause(2);
    end

end

end