function success = pickAndPlace(obj, x, y, z, phi, approach_dist, grip_width, pause_dur)
    % Pregrasp -> open gripper -> grasp -> close gripper -> retract to pregrasp.
    % pause_dur (s) is inserted between every motion segment so the user can
    % visually verify each step and the servos settle before the next move.

    if nargin < 8 || isempty(pause_dur),  pause_dur = 1.0; end
    if nargin < 7,                          grip_width = []; end
    if nargin < 6 || isempty(approach_dist), approach_dist = 5; end
    if nargin < 5 || (isnumeric(phi) && isnan(phi))
        phi = atan2(z - 24.2, sqrt(x^2 + y^2));
    end

    success = false;
    pregrasp_z = z + approach_dist;

    fprintf('  [pick] moving to pregrasp (%.1f, %.1f, %.1f)\n', x, y, pregrasp_z);
    if ~obj.moveByCoordinates(x, y, pregrasp_z, phi), return; end
    pause(pause_dur);

    obj.ungrip();
    pause(pause_dur);

    fprintf('  [pick] descending to grasp (%.1f, %.1f, %.1f)\n', x, y, z);
    if ~obj.moveByCoordinates(x, y, z, phi), return; end
    pause(pause_dur);

    obj.grip(grip_width);
    pause(pause_dur);

    fprintf('  [pick] retracting to pregrasp (%.1f, %.1f, %.1f)\n', x, y, pregrasp_z);
    if ~obj.moveByCoordinates(x, y, pregrasp_z, phi), return; end
    pause(pause_dur);

    success = true;
end
