function validSolutions = findValidSolution(obj, solutions)
    validSolutions = [];

    for i = 1:size(solutions, 1)

        disp(solutions(i,:))

        % --- Joint limit check ---
        if ~obj.checkJointLimits(solutions(i,:))
            if obj.show_debug_output
                fprintf('  Solution %d: outside joint limits\n', i);
            end
            continue
        end

        % --- Self collision check ---
        if obj.checkSelfCollision(solutions(i,:))
            if obj.show_debug_output
                fprintf('  Solution %d: self collision detected\n', i);
            end
            continue
        end

        % --- Floor collision check via FK ---
        [~, Ts] = obj.DH(solutions(i,:));
        endZ = Ts{end}(3, 4);
        if endZ < 0
            if obj.show_debug_output
                fprintf('  Solution %d: below floor (z=%.2f)\n', i, endZ);
            end
            continue
        end

        validSolutions = [validSolutions; solutions(i,:)];
    end

    if isempty(validSolutions) && obj.show_debug_output
        fprintf('  findValidSolution: no solutions passed all checks\n');
    end
end
