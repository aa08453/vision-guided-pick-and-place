function [sliced_rgb, model] = find_plane(pointCloud)
    % FIND_PLANE  RANSAC-fit the dominant horizontal plane and return an RGB
    % image of everything ABOVE it (table removed).
    %
    % Adapts to the actual organized-point-cloud dimensions instead of
    % assuming 480x640, so a 1080x1920 RealSense stream (or any other size)
    % works without out-of-bounds indexing into pointCloud.Color.

    maxDistance        = 0.01;        % 1 cm
    referenceVector    = [0, 0, 1];   % expect floor normal ~ camera Z
    maxAngularDistance = 5;           % degrees

    [model, ~, outlierIndices] = pcfitplane(pointCloud, ...
        maxDistance, referenceVector, maxAngularDistance);

    if ~isnumeric(pointCloud.Color) || ndims(pointCloud.Color) ~= 3
        error('find_plane: pointCloud.Color must be HxWx3 (organized cloud).');
    end
    [H, W, ~] = size(pointCloud.Color);

    sliced_rgb = uint8(zeros(H, W, 3));
    [row, col] = ind2sub([H, W], outlierIndices);
    for k = 1:length(outlierIndices)
        sliced_rgb(row(k), col(k), :) = uint8(pointCloud.Color(row(k), col(k), :));
    end
end
