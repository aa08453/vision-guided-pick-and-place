function collision = checkSelfCollision(obj, candidateConfig)
    % Interpolates from current joint angles to candidateConfig
    % and checks for self-collision at each intermediate step.
    % Skips first and last waypoints (start is known-safe,
    % end is caught by joint limit / reachability checks).

    N = 20;
    collision = false;

    initialConfig = obj.jointAngles(1:4);
    t = linspace(0, 1, N);

    for k = 2:length(t)-1
        config_k = initialConfig + t(k) * (candidateConfig - initialConfig);

        collision = checkCollision(obj.robotStructure, config_k, ...
            'SkippedSelfCollisions', 'parent');

        if collision
            if obj.show_debug_output
                fprintf('  checkSelfCollision: collision at t=%.2f (step %d/%d)\n', ...
                    t(k), k, N);
            end
            return
        end
    end
end
