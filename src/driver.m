% Drives the arm to four points on the circumference of a circle
clc; clear; close all;

% -------------------------------------------------------------------------
% Mode — swap between 'real' and 'sim'
% -------------------------------------------------------------------------
% arm = Arm('real', '/dev/ttyUSB0');
arm = Arm('sim');

% -------------------------------------------------------------------------
% Display / debug flags
%   show_four_IK               — overlay all 4 IK candidate arms in the
%                                visualizer (colored ghost arms)
%   show_debug_output          — print IK math and solution scoring to console
%   show_trajectory_interpolated — (reserved) animate interpolated path
% -------------------------------------------------------------------------
arm.show_four_IK                 = true;
arm.show_debug_output            = false;
arm.show_trajectory_interpolated = true;

% -------------------------------------------------------------------------
% Target trajectory — circle of radius r at height z
% -------------------------------------------------------------------------
r   = 31;   % radius (cm)
z   = 13.7; % fixed height (cm)
phi = NaN;  % NaN = auto-compute pitch, or fix e.g. -pi/2

angles_deg = [45, 135, 225, 315];
targets = [r*cosd(angles_deg); r*sind(angles_deg)]';  % 4x2

% -------------------------------------------------------------------------
% Run circle trajectory
% -------------------------------------------------------------------------
for i = 1:size(targets, 1)
    x = targets(i, 1);
    y = targets(i, 2);

    fprintf('Moving to point %d: (%.1f, %.1f, %.1f)\n', i, x, y, z);

    success = arm.moveByCoordinates(x, y, z, phi);

    if ~success
        fprintf('  WARNING: point %d unreachable — skipping\n', i);
        continue
    end

    fprintf('  Joint angles: %s\n', num2str(arm.jointAngles, '%.3f '));
    pause(2.0);
end

% -------------------------------------------------------------------------
% Example: set joint angles directly (radians, DH convention)
% Sends to hardware if real, always updates the visualizer
% -------------------------------------------------------------------------
% arm.moveByJoints([0, pi/4, -pi/4, 0]);

% -------------------------------------------------------------------------
% Save end-effector path plot
% -------------------------------------------------------------------------
% arm.savePlot('circle_run.png');
% fprintf('Done. Plot saved.\n');
