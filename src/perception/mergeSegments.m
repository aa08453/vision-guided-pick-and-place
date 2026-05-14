% function [mergedMasks, sortedLabels] = mergeSegments(sliced_rgb, segmented_rgb)
%     numClusters = 6;
%     dominantColors = zeros(numClusters, 3); % store mean RGB
% 
%     for i = 1:numClusters
%         mask = (segmented_rgb == i); % pixels belonging to cluster i
%         for c = 1:3
%             channel = sliced_rgb(:,:,c);
%             dominantColors(i,c) = mean(channel(mask)); % mean RGB of cluster
%         end
%     end
% 
%     y = [216 186 82];
%     r = [159 99 99];
%     g = [50 128 115];
%     b = [27 128 179];
%     grey = [98 86 68];
%     background = [0 0 0];
% 
%     referenceColors = double([y; r; g; b; grey; background]);
%     labels = ["yellow","red","green","blue","grey","background"];
% 
%     % mergedMasks = false(480, 640, 4); 
%     [H,W, ~] = size(sliced_rgb);
%     mergedMasks = false(H, W, 4);
%     clusterClassifications = strings(numClusters, 1);
%     for i = 1:numClusters
%         dists = vecnorm(referenceColors - dominantColors(i,:), 2, 2);
%         [~, refIdx] = min(dists); % refIdx is the index in our referenceColors list
% 
%         clusterClassifications(i) = labels(refIdx);
% 
%         % If the index is between 1 and 4 (Yellow, Red, Green, Blue)
%         if refIdx <= 4
%             % Add this K-means cluster to the corresponding fixed layer
%             clusterMask = (segmented_rgb == i);
%             mergedMasks(:,:,refIdx) = mergedMasks(:,:,refIdx) | clusterMask;
%         end
%     end
% 
% 
%     sortedLabels = strings(0);
%     for layer = 1:4
%         % Check if this color layer actually has pixels (objects found)
%         if any(reshape(mergedMasks(:,:,layer), [], 1))
%             sortedLabels = [sortedLabels; labels(layer)];
%         end
%     end
% 
% 
% end


function [mergedMasks, sortedLabels] = mergeSegments(sliced_rgb, segmented_rgb)
    % MERGESEGMENTS Merge K-means clusters into color categories
    %
    % This function combines color-based clustering with spatial distance metrics
    % to improve color classification. It considers:
    %   1. RGB color distance (default metric)
    %   2. Spatial depth distance (how close objects are physically)
    %
    % Parameters:
    %   sliced_rgb    - RGB image (H x W x 3)
    %   segmented_rgb - K-means cluster labels (H x W)
    %
    % Returns:
    %   mergedMasks   - 4-layer mask for [Yellow, Red, Green, Blue]
    %   sortedLabels  - String array of detected colors
    
    numClusters = 6;
    
    % Extract dominant colors for each cluster
    dominantColors = zeros(numClusters, 3);
    
    for i = 1:numClusters
        mask = (segmented_rgb == i);
        
        for c = 1:3
            channel = sliced_rgb(:, :, c);
            if any(mask(:))
                dominantColors(i, c) = mean(channel(mask));
            else
                dominantColors(i, c) = 0;
            end
        end
    end
    
    % Define reference colors (RGB values)
    % [referenceColors, labels] = get_reference_colors();
    y = [216 186 82];        % Yellow
    r = [159 99 99];         % Red
    g = [50 128 115];        % Green
    b = [27 128 179];        % Blue
    grey = [98 86 68];       % Grey
    background = [0 0 0];    % Background (black)
    
    referenceColors = double([y; r; g; b; grey; background]);
    labels = ["yellow"; "red"; "green"; "blue"; "grey"; "background"];
    
    % Get image dimensions
    [H, W, ~] = size(sliced_rgb);
    
    % Initialize output
    mergedMasks = false(H, W, 4);
    clusterClassifications = strings(numClusters, 1);
    clusterCentroids = [];  % Store centroids for spatial distance calculation
    
    % Calculate centroid for each cluster (for spatial distance)
    for i = 1:numClusters
        mask = (segmented_rgb == i);
        if any(mask(:))
            [y_coords, x_coords] = find(mask);
            clusterCentroids(i, :) = [mean(x_coords), mean(y_coords)];
        else
            clusterCentroids(i, :) = [0, 0];
        end
    end
    
    % Classify each cluster
    for i = 1:numClusters
        % Calculate color distance to all reference colors
        color_dists = vecnorm(referenceColors - dominantColors(i, :), 2, 2);
        
        % Calculate spatial distance to all reference colors (camera proximity)
            mask = (segmented_rgb == i);
    
        if ~any(mask(:))
            spatial_dists = zeros(6, 1);
            continue;
        end
        
        % Convert to grayscale to estimate depth/brightness
        gray_image = rgb2gray(sliced_rgb);
        
        % Get intensity values for this cluster
        cluster_intensities = gray_image(mask);
        mean_intensity = mean(cluster_intensities);
        
        % Get reference intensities (from reference colors)
        ref_gray = [
            mean([216, 186, 82]);      % Yellow intensity
            mean([159, 99, 99]);       % Red intensity
            mean([50, 128, 115]);      % Green intensity
            mean([27, 128, 179]);      % Blue intensity
            mean([98, 86, 68]);        % Grey intensity
            mean([0, 0, 0])            % Background intensity
        ];
        
        % Compute spatial distance as intensity difference
        spatial_dists = abs(ref_gray - mean_intensity);
        
        % Combine distances with weighted metric
        color_dists_norm = color_dists / max(color_dists + 1e-10);
        spatial_dists_norm = spatial_dists / max(spatial_dists + 1e-10);
        
        % Weighted combination
        color_weight = 0.75;      % 75% weight to color distance
        spatial_weight = 0.25;    % 25% weight to spatial distance
        
        combined_dists = color_weight * color_dists_norm + spatial_weight * spatial_dists_norm;
        
        % Find best matching reference color
        [~, refIdx] = min(combined_dists);
        clusterClassifications(i) = labels(refIdx);
        
        % Add to merged masks if it's a color (not grey or background)
        if refIdx <= 4
            clusterMask = (segmented_rgb == i);
            mergedMasks(:, :, refIdx) = mergedMasks(:, :, refIdx) | clusterMask;
        end
    end
    sortedLabels = [];
    % Extract detected colors
    for layer = 1:4
        if any(mergedMasks(:,:,layer), 'all')
            sortedLabels = [sortedLabels; labels(layer)];
        end
    end
end












