function servoAngles = sim2real(dhAngles)
% dh2servo - Convert DH kinematic angles back to servo command angles

    servoAngles = dhAngles;
    servoAngles(2) = servoAngles(2) - pi/2;
    servoAngles = wrapToPi(servoAngles);  % or wrapTo180 if you prefer degrees
end