function [collision, config_k] = checkSelfCollision(robot, jointAngleInitial, jointAngleFinal)
    N = 20;
    collision = false;
    config_k = jointAngleInitial;
    t = linspace(0,1,N);
    for k = 2:length(t)-1
        config_k = jointAngleInitial + t(k) * (jointAngleFinal - jointAngleInitial);
        collision = checkCollision(robot, config_k, 'SkippedSelfCollisions','parent');
         warning("Self collisions bro");

        if collision
            return
        end
    end
end