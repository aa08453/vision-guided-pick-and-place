function [segmented_rgb, sortedLabels] = vision_pipeline(pointCloud)
    % Main pipeline orchestrator
    % Processes point cloud through segmentation and instance detection
    
    % Create figure with white background and larger size
    % fig = figure('Color', 'white', 'Position', [100, 100, 1600, 1200]);
    
    % Stage 1: Extract plane and get image
    [sliced_rgb, model] = find_plane(pointCloud);
    % visualize_step(sliced_rgb, 1, 'Original Image');

    targetSize = [480 480];
    r = centerCropWindow2d(size(sliced_rgb),targetSize);
    rgb_crop = imcrop(sliced_rgb, r);
    % figure;
    % imshow(rgb_crop);
    % [x,y] = ginput;
    % display(x);
    % display(y);
    % figure;
    mask = true(size(rgb_crop));
    mask(130:310, 120:290, :) = false; % Mask a corner
    rgb_crop(~mask) = 0; 
    % 
    % imshow(rgb_crop)
    
    % Stage 2: Segment image into color regions
    numClusters = 6;
    [segmented_rgb, Centers] = segment(rgb_crop, numClusters);
    
    % Stage 3: Merge segments
    [mergedMasks, sortedLabels] = mergeSegments(rgb_crop, segmented_rgb);
    % visualize_merged_segments(mergedMasks, 2, 3);
    
    % Stage 4: Perform watershed for instance segmentation
    instanceMap = perform_watershed_segmentation(rgb_crop, mergedMasks);
    display(instanceMap)
    % visualize_step(label2rgb(instanceMap), 4, 'Labelled image');
    % figure;
    % imshow(instanceMap)
    % Stage 5: Extract and visualize object properties
    stats = extract_object_properties(instanceMap, 50); % 50 pixel area threshold
    display(stats)
    visualize_objects_with_axes(instanceMap, stats, 5);
    
    % Stage 6: Visualize poses in 3D
    visualize_3d_poses(pointCloud, instanceMap, stats, 6);
end
 
