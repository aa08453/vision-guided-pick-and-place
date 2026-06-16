function bestSolution = findSolution(obj, validSolutions)
    bestSolution = [];

    if isempty(validSolutions)
        warning('Arm.findSolution: no valid solutions to choose from');
        return
    end

    currentConfig = obj.jointAngles(1:4);
    weights       = [0.3, 0.3, 0.2, 0.2];
    bestCost      = inf;
    bestSolution  = validSolutions(1,:);

    for i = 1:size(validSolutions, 1)
        delta = abs(validSolutions(i,:) - currentConfig);
        cost  = weights * delta';
        if obj.show_debug_output
            fprintf('  Solution %d cost: %.4f\n', i, cost);
        end
        if cost < bestCost
            bestCost     = cost;
            bestSolution = validSolutions(i,:);
        end
    end

    if obj.show_debug_output
        fprintf('  Best: [%s]  cost: %.4f\n', num2str(bestSolution, '%.3f '), bestCost);
    end
end
