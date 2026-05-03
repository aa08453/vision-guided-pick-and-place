% This will move either the simulated robot or the real robot to the desired XYZ position
function success = moveByCoordinates(obj, x, y, z, phi)

	success = false;

	if nargin < 5 || (isnumeric(phi) && isnan(phi))
		phi = atan2(z - 24.2, sqrt(x^2 + y^2));
	end

	obj.ensurePlot();
	prevAngles = obj.jointAngles(1:4);

	solutions      = obj.findJointAngles(x, y, z, phi);
	obj.lastIKSolutions = solutions;
	validSolutions = obj.findValidSolution(solutions);
	bestSolution   = obj.findSolution(validSolutions);

	if isempty(bestSolution)
		return
	end

	if ~obj.isSimulated
		obj.arb.setpos(4, bestSolution(4), obj.speed);
		obj.arb.setpos(3, bestSolution(3), obj.speed);
		obj.arb.setpos(2, bestSolution(2) + pi/2, obj.speed);
		obj.arb.setpos(1, bestSolution(1), obj.speed);
		pause(obj.speed / 100);
	end

	obj.jointAngles(1:4) = bestSolution;
	obj.x_current  = x;
	obj.y_current  = y;
	obj.z_current  = z;
	obj.phi_current = phi;

	if obj.show_trajectory_interpolated
		obj.animateInterpolation(prevAngles, bestSolution);
	else
		obj.updatePlot();
	end

	success = true;
end
