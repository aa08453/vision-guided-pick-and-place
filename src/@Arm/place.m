function success = place(obj, x, y, z, phi, approach_dist, pause_dur)
    % Move to pre-place -> descend -> release -> retract to pre-place.
    % pause_dur (s) is inserted between every motion segment.

    if nargin < 7 || isempty(pause_dur),     pause_dur = 1.0; end
    if nargin < 6 || isempty(approach_dist), approach_dist = 5; end
    if nargin < 5 || (isnumeric(phi) && isnan(phi))
        phi = atan2(z - 24.2, sqrt(x^2 + y^2));
    end

    success = false;
    preplace_z = z + approach_dist;

    fprintf('  [place] moving to pre-place (%.1f, %.1f, %.1f)\n', x, y, preplace_z);
    if ~obj.moveByCoordinates(x, y, preplace_z, phi), return; end
    pause(pause_dur);

    fprintf('  [place] descending to place (%.1f, %.1f, %.1f)\n', x, y, z);
    if ~obj.moveByCoordinates(x, y, z, phi), return; end
    pause(pause_dur);

    obj.ungrip();
    pause(pause_dur);

    fprintf('  [place] retracting to pre-place (%.1f, %.1f, %.1f)\n', x, y, preplace_z);
    if ~obj.moveByCoordinates(x, y, preplace_z, phi), return; end
    pause(pause_dur);

    success = true;
end
