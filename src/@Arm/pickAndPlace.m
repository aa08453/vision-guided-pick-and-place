function success = pickAndPlace(obj, x, y, z, phi, approach_dist, grip_width)
    % Pregrasp -> open gripper -> grasp -> close gripper -> retract to pregrasp
    %
    % grip_width (cm, optional): target finger spread when clamping.
    %   Pass the cube's short-axis width minus ~0.5-1 cm for a firm grip.
    %   Omit (or pass [] / NaN) for full close (backwards-compatible).

    if nargin < 7, grip_width = []; end
    if nargin < 6 || isempty(approach_dist), approach_dist = 5; end
    if nargin < 5 || (isnumeric(phi) && isnan(phi))
        phi = atan2(z - 24.2, sqrt(x^2 + y^2));
    end

    success = false;
    pregrasp_z = z + approach_dist;

    fprintf('  [pick] moving to pregrasp (%.1f, %.1f, %.1f)\n', x, y, pregrasp_z);
    if ~obj.moveByCoordinates(x, y, pregrasp_z, phi), return; end

    obj.ungrip();
    pause(0.3);

    fprintf('  [pick] descending to grasp (%.1f, %.1f, %.1f)\n', x, y, z);
    if ~obj.moveByCoordinates(x, y, z, phi), return; end

    obj.grip(grip_width);
    pause(0.3);

    fprintf('  [pick] retracting to pregrasp (%.1f, %.1f, %.1f)\n', x, y, pregrasp_z);
    if ~obj.moveByCoordinates(x, y, pregrasp_z, phi), return; end

    success = true;
end
