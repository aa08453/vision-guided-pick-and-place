

function instanceMap = perform_watershed_segmentation(sliced_rgb, mergedMasks)
    % Perform marker-controlled watershed for instance segmentation
    % Returns instanceMap with unique IDs for each instance
    
    instanceMap = zeros(size(mergedMasks, 1), size(mergedMasks, 2));
    currentID = 1;
    
    % Convert to LAB color space for color-based gradient
    lab_img = rgb2lab(sliced_rgb);
    a_chan = lab_img(:, :, 2);
    b_chan = lab_img(:, :, 3);
    % imshow(lab_img)
    
    % Process each color segment
    for c = 1:4 % y r g b grey bg
        bw = mergedMasks(:, :, c);
        if ~any(bw(:)), continue; end
        
        % Select relevant color channel for gradient
        if c == 1 || c == 4  % Yellow or Blue
            target_chan = b_chan;
        else  % Red or Green
            target_chan = a_chan;
        end
        
        % Calculate gradients
        color_gradient = imgradient(target_chan);
        intensity_gradient = imgradient(rgb2gray(sliced_rgb));
        total_gradient = color_gradient + intensity_gradient;
        
        % Marker-controlled watershed
        [currentID, instanceMap] = watershed_segment_region(bw, total_gradient, instanceMap, currentID);
    end
end
