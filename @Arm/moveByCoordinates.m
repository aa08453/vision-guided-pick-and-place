% This will move either the simulated robot or the real robot to the desired XYZ position
function success = moveByCoordinates(obj, x, y, z, phi)

	success = false;
	% phi is optional — pass NaN or omit to auto-compute

	% fprintf('  DEBUG: entered moveByCoordinates\n');

	if nargin < 5 || (isnumeric(phi) && isnan(phi))
		phi = atan2(z - 24.2, sqrt(x^2 + y^2));
	end
	% fprintf('  DEBUG: phi = %.4f\n', phi);
	obj.ensurePlot();
	% fprintf('  DEBUG: ensurePlot done\n');
	% Get current joint angles as warm start for IK
	q0 = obj.jointAngles(1:4);

	

	solutions      = obj.findJointAngles(x, y, z, phi);
	% fprintf('  DEBUG: findJointAngles returned %d solutions\n', size(solutions,1));
	validSolutions = obj.findValidSolution(solutions);
	% fprintf('  DEBUG: findValidSolution returned %d valid\n', size(validSolutions,1));
	bestSolution   = obj.findSolution(validSolutions);
	% fprintf('  DEBUG: findSolution done\n');
	if isempty(bestSolution)
	    success = false;
	    return
	end

	% Send to hardware if real
	if ~obj.isSimulated
		obj.sendJointsToHardware(bestSolution);
		pause(obj.speed / 100);  % scale pause by speed
	end

	% Update state

	% fprintf('  DEBUG: updating joints to [%s]\n', num2str(bestSolution, '%.3f '));

	obj.jointAngles(1:4) = bestSolution;
	obj.x_current = x;
	obj.y_current = y;
	obj.z_current = z;
	obj.phi_current = phi;


	% fprintf('  DEBUG: joint angles now [%s]\n', num2str(obj.jointAngles, '%.3f '));


	obj.updatePlot();
	success = true;
end

