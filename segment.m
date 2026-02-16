function [segmented_rgb, Centers, numClusters] = segment(sliced_rgb)  
    lab_img = rgb2lab(sliced_rgb);
    a = lab_img(:,:,2);
    b = lab_img(:,:,3);
    
    % 2. Filter out the "black" background (near 0,0,0 in Lab)
    mask = sum(sliced_rgb, 3) > 20; 
    a_vals = a(mask);
    b_vals = b(mask);
    
    % 3. Generate a 2D Histogram (edges roughly -100 to 100)
    edges = -110:2:110;
    counts = histcounts2(a_vals, b_vals, edges, edges);
    
    % 4. Smooth and Find Peaks
    % Using imregionalmax or finding local maximums
    counts_smoothed = imgaussfilt(counts, 1.5); 
    peaks = imregionalmax(counts_smoothed);
    numClusters = sum(peaks(:));  

    imshow(sliced_rgb);
    [segmented_rgb, Centers] = imsegkmeans(uint8(sliced_rgb), numClusters, numAttempts=3);
    % L = segmented_rgb;
    % figure;
    % imshow(label2rgb(segmented_rgb))
end