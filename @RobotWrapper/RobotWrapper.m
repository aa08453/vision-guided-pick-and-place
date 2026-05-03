% This has the following attributes
% Real or simulated robot


% There is a config struct which contains:
%	- Whether to show 4 IK solutions (show_four_IK)
% 	- Whether to show the trajectory being interpolated (show_trajectory_interpolated)
% 	- Show debug output for the IK solutions (show_debug_output)
%	- Whether 


% Robot DH parameters

% These are probably better stored as methods, if a real robot, then calculate it and update the private variable, and return if a sim robot then just read from before
% Current joint angles 
% Current FK coordinates
% Current commanded IK
