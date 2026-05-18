function cubes = snapshot_cubes(ptCloud, T_cam_to_robot, opts)
    % SNAPSHOT_CUBES  Detect cubes in a point cloud and report robot-frame pose.
    %
    %   cubes = snapshot_cubes(ptCloud, T_cam_to_robot)
    %   cubes = snapshot_cubes(ptCloud, T_cam_to_robot, opts)
    %
    % opts (struct, all optional):
    %   .show_debug    show segmentation figures (default false)
    %   .min_pts_3d    minimum valid 3D points required to keep a cube (default 30)
    %
    % Returns a struct array. Each element has:
    %   .color       string  -- currently always "any" while color logic
    %                          is commented out in hsv_segment.m.
    %                          Originally "yellow" | "red" | "green" | "blue".
    %   .pos_cam     1x3     centroid in camera frame (meters)
    %   .pos_robot   1x3     centroid in robot frame (cm)
    %   .yaw_robot   scalar  major-axis direction in robot XY plane (rad)
    %   .minor_cm    scalar  cube extent along its 2D minor axis (cm)
    %   .major_cm    scalar  cube extent along its 2D major axis (cm)
    %   .height_cm   scalar  top-face height above the table (cm, robot Z)
    %   .num_points  scalar  number of valid 3D points used
    %
    % The detection front-end mirrors vision_pipeline.m: RANSAC plane removal,
    % 480x480 center crop, HSV + Area/Solidity segmentation. Each surviving
    % connected component becomes one cube entry.

    if nargin < 3, opts = struct(); end
    if ~isfield(opts, 'show_debug'), opts.show_debug = false; end
    if ~isfield(opts, 'min_pts_3d'), opts.min_pts_3d = 30;    end

    % ---- Segmentation -------------------------------------------------------
    [sliced_rgb, ~] = find_plane(ptCloud);
    [origH, origW, ~] = size(sliced_rgb);

    targetSize = [480 480];
    r = centerCropWindow2d(size(sliced_rgb), targetSize);
    rgb_crop = imcrop(sliced_rgb, r);

    [mergedMasks_crop, instanceMap_crop, ~, idColors] = hsv_segment(rgb_crop);

    if opts.show_debug
        figure; imshow(rgb_crop); title('snapshot_cubes: taskspace crop');
        visualize_merged_segments(mergedMasks_crop);
    end

    [cropH, cropW] = size(instanceMap_crop);
    rowOffset = floor((origH - cropH) / 2);
    colOffset = floor((origW - cropW) / 2);
    instanceMap = zeros(origH, origW);
    instanceMap(rowOffset + (1:cropH), colOffset + (1:cropW)) = instanceMap_crop;

    % ---- Per-instance 3D pose extraction -----------------------------------
    cubes = struct('color', {}, 'pos_cam', {}, 'pos_robot', {}, ...
                   'yaw_robot', {}, 'minor_cm', {}, 'major_cm', {}, ...
                   'height_cm', {}, 'num_points', {});

    numInstances = max(instanceMap(:));
    if numInstances == 0, return; end

    pcLoc = ptCloud.Location;
    organized = (ndims(pcLoc) == 3);

    for k = 1:numInstances
        mask = (instanceMap == k);
        if ~any(mask, 'all'), continue; end

        if organized
            X = pcLoc(:,:,1); Y = pcLoc(:,:,2); Z = pcLoc(:,:,3);
            idx = find(mask);
            pts_cam = [X(idx), Y(idx), Z(idx)];
        else
            if numel(mask) ~= size(pcLoc, 1), continue; end
            pts_cam = pcLoc(mask(:), :);
        end

        valid = ~any(isnan(pts_cam), 2) & ~any(isinf(pts_cam), 2) ...
                & pts_cam(:, 3) > 0;
        pts_cam = pts_cam(valid, :);
        if size(pts_cam, 1) < opts.min_pts_3d, continue; end

        centroid_cam = mean(pts_cam, 1);
        pts_robot    = cam_to_robot(pts_cam, T_cam_to_robot);
        centroid_rob = mean(pts_robot, 1);

        % PCA on the XY projection -> orthogonal axes and their extents
        xy = pts_robot(:, 1:2);
        xy_c = xy - mean(xy, 1);
        C = (xy_c' * xy_c) / max(1, size(xy, 1) - 1);
        [V, D] = eig(C);
        [eigvals, ord] = sort(diag(D), 'descend'); %#ok<ASGLU>
        V = V(:, ord);

        proj_major = xy_c * V(:, 1);
        proj_minor = xy_c * V(:, 2);
        major_cm = max(proj_major) - min(proj_major);
        minor_cm = max(proj_minor) - min(proj_minor);

        cubes(end+1) = struct( ...                                       %#ok<AGROW>
            'color',      idColors(k), ...
            'pos_cam',    centroid_cam, ...
            'pos_robot',  centroid_rob, ...
            'yaw_robot',  atan2(V(2,1), V(1,1)), ...
            'minor_cm',   minor_cm, ...
            'major_cm',   major_cm, ...
            'height_cm',  max(pts_robot(:, 3)), ...
            'num_points', size(pts_cam, 1));
    end
end
