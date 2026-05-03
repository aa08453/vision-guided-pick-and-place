% This has the following attributes
% Real or simulated robot

% Robot DH parameters

% These are probably better stored as methods, if a real robot, then calculate it and update the private variable, and return if a sim robot then just read from before
% Current joint angles 
% Current FK coordinates
% Current commanded IK

% Public Variables:
%	- Whether to show 4 IK solutions (show_four_IK)
% 	- Whether to show the trajectory being interpolated (show_trajectory_interpolated)
% 	- Show debug output for the IK solutions (show_debug_output)
% 	- X, Y, Z
% 	- Current pitch (as phi)
%	- Joint angles
%	- Gripping Status

% Private Variables 
%	- 


classdef Arm
	properties  (Access = public)
		show_four_IK;
		show_trajectory_interpolated;
		show_debug_output;
		X;
		Y;
		z;
		phi;
		jointAngles = [theta_1, theta_2, theta_3, theta_4, theta_5];
		grippingStatus;
		speed;
		com_port;
		grip_closeness; 
	end

	properties (Access = private)
		robotStructure;
	end



	methods (Access = public)
		[] = moveByJoints(jointAnglesCommanded);
		[] = moveByCoordinates(X,Y,Z,phi); % if phi not given then autocompute phi?
		[] = grip();
		[] = ungrip();
		[] = forward();
	end

	methods (Access = private)
		% FK functions
		[T,Ts] = DH(); %TODO: Try to change this to X,Y,Z,R

		% Helper for calculating IK
		[solutions] = findJointAngles(x,y,z,phi); 

		% Helpers for calculating intersections/boundaries
		[collision] = checkSelfCollision(solutions); 

		[withinLimits] = checkJointLimits(solution);

		[reachable] = isReachable(x,y,z,phi);


		% Helpers for calculating final solutions
		[validSolutions] = findValidSolution(solutions);
		[bestConfig] = inverse(validSolutions);




	end
end
