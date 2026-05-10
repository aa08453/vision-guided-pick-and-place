function success = place(obj, x, y, z, phi, approach_dist)
    % Move to pre-place → descend → release → retract to pre-place
    if nargin < 6 || isempty(approach_dist), approach_dist = 5; end
    if nargin < 5 || (isnumeric(phi) && isnan(phi))
        phi = atan2(z - 24.2, sqrt(x^2 + y^2));
    end

    success = false;
    preplace_z = z + approach_dist;

    fprintf('  [place] moving to pre-place (%.1f, %.1f, %.1f)\n', x, y, preplace_z);
    if ~obj.moveByCoordinates(x, y, preplace_z, phi), return; end

    fprintf('  [place] descending to place (%.1f, %.1f, %.1f)\n', x, y, z);
    if ~obj.moveByCoordinates(x, y, z, phi), return; end

    obj.ungrip();
    pause(0.3);

    fprintf('  [place] retracting to pre-place (%.1f, %.1f, %.1f)\n', x, y, preplace_z);
    if ~obj.moveByCoordinates(x, y, preplace_z, phi), return; end

    success = true;
end
