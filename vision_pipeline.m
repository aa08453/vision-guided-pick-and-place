function [segmented_rgb, sortedLabels] = vision_pipeline(pointCloud)
[sliced_rgb, model] = find_plane(pointCloud);
[segmented_rgb, Centers, numClusters] = segment(sliced_rgb); % Gives segment mask
[mergedMasks, sortedLabels] = mergeSegments(sliced_rgb, segmented_rgb);


weights = reshape([1, 2, 3, 4], [1, 1, 4]);
mergedSegments = sum(double(mergedMasks) .* weights, 3);


% Map 1->Yellow, 2->Red, 3->Green, 4->Blue
customColormap = [1 1 0;  % 1: Yellow
                  1 0 0;  % 2: Red
                  0 1 0;  % 3: Green
                  0 0 1]; % 4: Blue
rgbLabelImage = label2rgb(mergedSegments, customColormap, 'k'); % 'k' for black background
subplot(2,2,1);
imshow(rgbLabelImage);
title('Original image')
subplot(2,2,2);
imshow(sliced_rgb);
title('Collapsed 2D Label Map');

instanceMap = zeros(480, 640);
currentID = 1;

lab_img = rgb2lab(sliced_rgb);
a_chan = lab_img(:,:,2);
b_chan = lab_img(:,:,3);



for c = 1:4
    
    bw = mergedMasks(:,:,c);
    if ~any(bw(:)), continue; end
    
    % 2. Select the most relevant color channel for the gradient
    % Yellow and Blue are most active in 'b', Red and Green in 'a'
    if c == 1 || c == 4 % Yellow or Blue
        target_chan = b_chan;
    else % Red or Green
        target_chan = a_chan;
    end
    
    % 3. Calculate Gradient on the COLOR channel, not just intensity
    % This finds where "yellow-ness" or "blue-ness" changes
    [Gmag, ~] = imgradient(target_chan);
    
    % 4. Combine with the original intensity gradient to find shadows
    total_gradient = Gmag + imgradient(rgb2gray(sliced_rgb));
    
    % 5. Marker-controlled watershed
    D = -bwdist(~bw);
    % We use 'h-minima' to suppress noise. Increase 2.5 if still over-segmenting.
    markers = imextendedmin(D, 2.5); 
    W = imimposemin(total_gradient, markers);
    
    L = watershed(W);
    L(~bw) = 0;
    
    
    numObjects = max(L(:));
    for i = 1:numObjects
        mask = (L == i);
        instanceMap(mask) = currentID;
        currentID = currentID + 1;
    end
end
subplot(2,2,3);

imshow(label2rgb(instanceMap))
title('Object Detection')


CC = bwconncomp(instanceMap > 0);

stats = regionprops("table", CC, ...
    "Area",...
    "MajorAxisLength",...
    "MinorAxisLength", ...
    "Orientation", ...
    "PixelIdxList");

figure;

% individual objects
for k = 1:height(stats)
    % Extract the individual object mask
    objMask = (instanceMap == k);

    pts = pointCloud.Location;
    objPoints = pts(repmat(objMask, [1 1 3]));  
    objPoints = reshape(objPoints, [], 3);
    objPoints = objPoints(~any(isnan(objPoints),2), :); % remove NaNs
    centroid = mean(objPoints, 1);   % [x y z]
    stats = regionprops(objMask, 'Orientation');
    yaw = deg2rad(stats.Orientation);   % radians
    pose.yaw = yaw;
    Rz = [ cos(yaw) -sin(yaw) 0;
       sin(yaw)  cos(yaw) 0;
       0         0        1 ];

    pose.R = Rz;
    pose.d = centroid;
    T = [pose.R pose.d';
        [0 0 0] 1];
    display(T);
    pcshow(pointCloud, "VerticalAxisDir", "Up");
    hold on;
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
        
    p = pose.d;   % [X Y Z]
    
    plot3(p(1), p(2), p(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        
    L = 0.05;  % axis length (meters or point cloud units)
    
    x_axis = pose.R * [L;0;0];
    y_axis = pose.R * [0;L;0];
    z_axis = pose.R * [0;0;L];
    
    quiver3(p(1), p(2), p(3), x_axis(1), x_axis(2), x_axis(3), ...
        'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    quiver3(p(1), p(2), p(3), y_axis(1), y_axis(2), y_axis(3), ...
        'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    quiver3(p(1), p(2), p(3), z_axis(1), z_axis(2), z_axis(3), ...
        'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
end

hold off

end