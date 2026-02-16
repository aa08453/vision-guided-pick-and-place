function [segmented_rgb, Centers, numClusters] = segment(sliced_rgb)  
    ds_factor = 1; 
    plane_small = imresize(sliced_rgb, 1/ds_factor, 'nearest');
    [rows_s, cols_s, ~] = size(plane_small);
    
    pixels_s = double(reshape(plane_small, [], 3));
    
    valid_mask = sum(pixels_s, 2) > 15; 
    pixelData = pixels_s(valid_mask, :);
    
    epsilon = 7;    % Search radius in RGB space
    minpts = 400;   % Minimum pixels to form a cluster at 1/4 resolution
    idx = dbscan(pixelData, epsilon, minpts);
    
    L_small = zeros(rows_s * cols_s, 1);
    L_small(valid_mask) = idx;
        
    foundClusters = max(idx);
    fprintf('DBSCAN found %d actual colored objects.\n', foundClusters);
    
    numClusters = foundClusters;
    imshow(sliced_rgb);
    [segmented_rgb, Centers] = imsegkmeans(uint8(sliced_rgb), numClusters, numAttempts=3);
    % L = segmented_rgb;
    % figure;
    % imshow(label2rgb(segmented_rgb))
end