% Demonstrates IK display, circle trajectory, and pick-and-place with a cube
clc; clear; close all;

% -------------------------------------------------------------------------
% Mode — swap between 'real' and 'sim'
% -------------------------------------------------------------------------
arm = Arm('real', 'COM4');
% arm = Arm('sim');

% -------------------------------------------------------------------------
% Display / debug flags
%   show_four_IK               — IK solution table in the annotation panel
%   show_debug_output          — print IK math and solution scoring to console
%   show_trajectory_interpolated — animate interpolated path
% -------------------------------------------------------------------------
arm.show_four_IK                 = true;
arm.show_debug_output            = false;
arm.show_trajectory_interpolated = true;

% =========================================================================
% Part 1 — Circle trajectory (verifies IK at all four quadrants)
% =========================================================================
% fprintf('\n=== Part 1: circle trajectory ===\n');

% arm.moveByCoordinates(0,0,45.7,NaN);

% r   = 21;   % radius (cm)
% z   = 5; % fixed height (cm)
% phi = NaN;  % NaN = auto-compute pitch
% 
% angles_deg = [45, 135, 225, 315];
% targets = [r*cosd(angles_deg); r*sind(angles_deg)]';  % 4x2
% 
% for i = 1:size(targets, 1)
%     x = targets(i, 1);
%     y = targets(i, 2);
%     fprintf('Moving to point %d (%.0f deg): (%.1f, %.1f, %.1f)\n', ...
%         i, angles_deg(i), x, y, z);
% 
%     success = arm.moveByCoordinates(x, y, z, phi);
% 
%     if ~success
%         fprintf('  WARNING: point %d unreachable — skipping\n', i);
%         continue
%     end
% 
%     fprintf('  Joint angles (rad): %s\n', num2str(arm.jointAngles, '%.3f '));
%     fprintf('  theta1 = %.1f deg\n', rad2deg(arm.jointAngles(1)));
%     pause(1.5);
% end

% =========================================================================
% Part 2 — Pick and place with a cube
% =========================================================================
fprintf('\n=== Part 2: pick and place ===\n');

% for i = 1:5

% Place the cube at the pick location
pick_x = 20; pick_y = 0; pick_z = 2;
arm.setCubePos(pick_x, pick_y, pick_z);
fprintf('Cube placed at (%.1f, %.1f, %.1f)\n', pick_x, pick_y, pick_z);
pause(1.0);

% Pick up the cube
fprintf('\nPicking cube at (%.1f, %.1f, %.1f) ...\n', pick_x, pick_y, pick_z);
success = arm.pickAndPlace(pick_x, pick_y, pick_z, -pi/2, 7);
if ~success
    fprintf('  WARNING: pick failed\n');
else
    fprintf('  Pick successful.\n');
    pause(1.0);
    arm.grip()

    % Place the cube at a new location
    place_x = -20; place_y = 0; place_z = 2;
    fprintf('\nPlacing cube at (%.1f, %.1f, %.1f) ...\n', place_x, place_y, place_z);
    success = arm.place(place_x, place_y, place_z, -pi/2, 7);
    if ~success
        fprintf('  WARNING: place failed\n');
    else
        fprintf('  Place successful.\n');
    end
end

fprintf('\nDone.\n');

arm.moveByCoordinates(0,0, 45.7,NaN);

pause(3)


pick_x = -20; pick_y = 0; pick_z = 2;
arm.setCubePos(pick_x, pick_y, pick_z);
fprintf('Cube placed at (%.1f, %.1f, %.1f)\n', pick_x, pick_y, pick_z);
pause(1.0);

% Pick up the cube
fprintf('\nPicking cube at (%.1f, %.1f, %.1f) ...\n', pick_x, pick_y, pick_z);
success = arm.pickAndPlace(pick_x, pick_y, pick_z, -pi/2, 7);
if ~success
    fprintf('  WARNING: pick failed\n');
else
    fprintf('  Pick successful.\n');
    pause(1.0);
    arm.grip()

    % Place the cube at a new location
    place_x = 20; place_y = 0; place_z = 2;
    fprintf('\nPlacing cube at (%.1f, %.1f, %.1f) ...\n', place_x, place_y, place_z);
    success = arm.place(place_x, place_y, place_z, -pi/2, 7);
    if ~success
        fprintf('  WARNING: place failed\n');
    else
        fprintf('  Place successful.\n');
    end
end

% pause(5)

% end
