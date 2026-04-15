function servoAngles = dh2servo(dhAngles, deg)
% dh2servo - Convert DH kinematic angles back to servo command angles

    servoAngles = dhAngles;
    
    % Remove the offset from joint 2
    if length(servoAngles) >= 2
        servoAngles(2) = servoAngles(2) - deg2rad(90);
    end
    
    if deg
        servoAngles = rad2deg(servoAngles);
    end
    
    servoAngles = wrapToPi(servoAngles);  % or wrapTo180 if you prefer degrees
end