function [segmented_rgb, Centers] = segment(sliced_rgb, numClusters)  
    imshow(sliced_rgb);
    [segmented_rgb, Centers] = imsegkmeans(uint8(sliced_rgb), numClusters, numAttempts=3);
    % L = segmented_rgb;
    % figure;
    % imshow(label2rgb(segmented_rgb))
end