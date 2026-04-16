function servoAngles = processDHForSim(dhAngles)
    servoAngles = dhAngles;
    servoAngles(2) = servoAngles(2) - pi/2;
end