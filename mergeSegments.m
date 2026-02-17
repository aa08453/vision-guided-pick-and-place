function [mergedMasks, sortedLabels] = mergeSegments(sliced_rgb, segmented_rgb)
    numClusters = 6;
    dominantColors = zeros(numClusters, 3); % store mean RGB
    
    for i = 1:numClusters
        mask = (segmented_rgb == i); % pixels belonging to cluster i
        for c = 1:3
            channel = sliced_rgb(:,:,c);
            dominantColors(i,c) = mean(channel(mask)); % mean RGB of cluster
        end
    end
    
    y = [135 51 0];
    r = [87 13 17];
    g = [1 37 5];
    b = [0 54 68];
    grey = [98 86 68];
    background = [0 0 0];
    
    referenceColors = double([y; r; g; b; grey; background]);
    labels = ["yellow","red","green","blue","grey","background"];

    mergedMasks = false(480, 640, 4); 
    clusterClassifications = strings(numClusters, 1);
    for i = 1:numClusters
        dists = vecnorm(referenceColors - dominantColors(i,:), 2, 2);
        [~, refIdx] = min(dists); % refIdx is the index in our referenceColors list
        
        clusterClassifications(i) = labels(refIdx);
        
        % If the index is between 1 and 4 (Yellow, Red, Green, Blue)
        if refIdx <= 4
            % Add this K-means cluster to the corresponding fixed layer
            clusterMask = (segmented_rgb == i);
            mergedMasks(:,:,refIdx) = mergedMasks(:,:,refIdx) | clusterMask;
        end
    end


    sortedLabels = strings(0);
    for layer = 1:4
        % Check if this color layer actually has pixels (objects found)
        if any(reshape(mergedMasks(:,:,layer), [], 1))
            sortedLabels = [sortedLabels; labels(layer)];
        end
    end


end