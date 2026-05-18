function [instanceMap, sortedLabels] = vision_pipeline(pointCloud)
    % Main pipeline orchestrator
    % Processes point cloud through plane removal, HSV color segmentation,
    % and per-color geometric filtering. Each surviving connected component
    % becomes one instance, so no watershed is needed.
    %
    % Returns:
    %   instanceMap  - H x W label image (unique ID per object), padded back
    %                  to the original point-cloud dims so compute_object_pose
    %                  can index pointCloud.Location with the mask directly.
    %   sortedLabels - string column of detected colors

    % Stage 1: RANSAC plane removal -- drops the table and any object resting
    % flush on it.
    [sliced_rgb, ~] = find_plane(pointCloud);
    visualize_step(sliced_rgb, 'Original Image');

    [origH, origW, ~] = size(sliced_rgb);

    % Stage 2: Center-crop to the robot's taskspace (480x480) before
    % segmentation. Everything outside the crop is forced to ID = 0, so
    % out-of-workspace objects (e.g., calibration markers on the table edge)
    % cannot be picked up as targets.
    targetSize = [640 640];
    r = centerCropWindow2d(size(sliced_rgb), targetSize);
    rgb_crop = imcrop(sliced_rgb, r);

    figure;
    imshow(rgb_crop)
    title('Taskspace crop (480x480) sent to HSV segmentation');

    % Stage 3: HSV color segmentation + Area/Solidity filtering. Each
    % surviving connected component gets a unique instance ID.
    [mergedMasks_crop, instanceMap_crop, sortedLabels] = hsv_segment(rgb_crop);

    % Pad cropped masks back to the original frame so logical indexing into
    % the organized point cloud (which is still origH x origW) lines up.
    % Pixels outside the crop window stay 0 / false and are ignored.
    [cropH, cropW] = size(instanceMap_crop);
    rowOffset = floor((origH - cropH) / 2);
    colOffset = floor((origW - cropW) / 2);

    mergedMasks = false(origH, origW, 4);
    mergedMasks(rowOffset + (1:cropH), colOffset + (1:cropW), :) = mergedMasks_crop;

    instanceMap = zeros(origH, origW);
    instanceMap(rowOffset + (1:cropH), colOffset + (1:cropW)) = instanceMap_crop;

    visualize_merged_segments(mergedMasks);

    % Stage 4: Extract and visualize object properties
    stats = extract_object_properties(instanceMap, 50); % 50 pixel area threshold
    visualize_objects_with_axes(instanceMap, stats);

    % Stage 5: Visualize poses in 3D
    visualize_3d_poses(pointCloud, instanceMap, stats);
end
