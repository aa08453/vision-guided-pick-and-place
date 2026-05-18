function [currentID, instanceMap] = watershed_segment_region(bw, total_gradient, instanceMap, currentID)
    % Apply watershed algorithm to segment a single region

    % Distance transform
    D = -bwdist(~bw);
    
    % Extended minima (suppress noise; increase 2.5 if over-segmenting)
    markers = imextendedmin(D, 4);
    
    % Impose minima on gradient
    W = imimposemin(total_gradient, markers);
    
    % Watershed
    L = watershed(W);
    L(~bw) = 0;
    
    % Assign unique IDs to instances
    numObjects = max(L(:));
    for i = 1:numObjects
        mask = (L == i);
        instanceMap(mask) = currentID;
        currentID = currentID + 1;
    end
end
 
