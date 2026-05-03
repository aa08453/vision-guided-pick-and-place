function [validSolutions, bestConfig] = findSolution(x,y,z,phi,robot,currentConfig)
    configs = findJointAngles(x,y,z,phi);
    validSolutions = [];
    
    for i = 1:size(configs,1)
        withinLimits = checkJointLimits(configs(i,:));
        if ~withinLimits
            disp("Not within limits");
            continue
        end
        [collision, ~] = checkSelfCollision(robot, currentConfig, configs(i,:));
        if collision
            disp("Collision detected");
            continue
        end
        % 

        % 
        % if (abs(configs(i,2) + configs(i,3) + configs(i,4)) < 1.9)
        %     disp("collision with the floor")
        %     continue
        % end
        [~,~,z, ~] = pincherFK(configs(i,:));
        if (z < 0)
            disp("collision with the floor")
            continue
        end
        validSolutions = [validSolutions; configs(i,:)];
    end
    
    if isempty(validSolutions)
        bestConfig = [];
        warning('No valid solutions found');
        return
    end

    
    b = [0.3, 0.3, 0.2, 0.2];
    bestCost = inf;
    bestConfig = validSolutions(1,:);

    for i = 1:size(validSolutions,1)
        delta = abs(validSolutions(i,:) - currentConfig);
        cost = b * delta';
        if cost < bestCost
            bestCost = cost;
            bestConfig = validSolutions(i,:);
        end
    end
end