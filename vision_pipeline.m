function vision_pipeline(pointCloud)
[sliced_rgb, params] = find_plane(pointCloud);
[segmented_rgb, Centers, numClusters] = segment(sliced_rgb)
imshow(label2rgb(segmented_rgb))


numColors = 4;
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

clusterLabels = strings(numClusters,1);

for i = 1:numClusters
    dists = vecnorm(referenceColors - dominantColors(i,:), 2, 2); % Euclidean distance
    [~, idx] = min(dists);
    clusterLabels(i) = labels(idx);
end

[sortedLabels, sortIdx] = sort(clusterLabels);

segmented_colors = uint8(zeros(480,640, numColors));

% Reorder clusters for display
for k = 1:numClusters
    i = sortIdx(k);
    mask = (segmented_rgb == i);
    
    segmented_img = zeros(size(sliced_rgb), 'uint8');
    for c = 1:3
        channel = sliced_rgb(:,:,c);
        channel(~mask) = 0;
        segmented_img(:,:,c) = channel;
    end
    subplot(3,2,k)
    imshow(segmented_img)
    title(sprintf("Cluster %d → %s", i, clusterLabels(i)))
    if clusterLabels(i) == "blue"
        segmented_colors(:,:,3) = rgb2gray(segmented_img);
    elseif clusterLabels(i) == "red"
        segmented_colors(:,:,1) = rgb2gray(segmented_img);
    elseif clusterLabels(i) == "yellow" 
        segmented_colors(:,:,4) = rgb2gray(segmented_img);
    elseif clusterLabels(i) == "green"
        segmented_colors(:,:,2) = rgb2gray(segmented_img);
    end
end

%s
X_layer = pointCloud.Location(:,:,1);
Y_layer = pointCloud.Location(:,:,2);
Z_layer = pointCloud.Location(:,:,3);

figure;

colorNames = ["Red","Green","Blue","Yellow"];



for c = 1:4
    
    binaryMask = segmented_colors(:,:,c) > 0;
    
    %s
    % 1. Fill small holes inside the cube
    binaryMask = imfill(binaryMask, 'holes');
    % 2. Remove tiny noise (anything smaller than 500 pixels)
    binaryMask = bwareaopen(binaryMask, 500); 
    % 3. Bridge small gaps between parts of the same cube
    se = strel('disk', 5);
    binaryMask = imclose(binaryMask, se);
    
    CC = bwconncomp(binaryMask);
    
    stats = regionprops("table", CC, ...
        "Area", "Centroid", "MajorAxisLength", ...
        "MinorAxisLength", "Orientation", "PixelIdxList"); %s

    %s
    if isempty(stats)
        fprintf('No object found for color: %s\n', colorNames(c));
        continue; 
    end
    
    %s
    % 1 co-ordinate per color
    [~, largestIdx] = max(stats.Area);
    pixelIdx = stats.PixelIdxList{largestIdx};
    
    subplot(2,2,c)
    imshow(binaryMask)
    title(colorNames(c))
    hold on
    
    for k = 1:height(stats)
        
        % Extract region properties
        x = stats.Centroid(k,1);
        y = stats.Centroid(k,2);
        major = stats.MajorAxisLength(k);
        minor = stats.MinorAxisLength(k);
        theta = deg2rad(-stats.Orientation(k));
        
        % ----- Centroid -----
        plot(x, y, 'go', 'MarkerSize', 6, 'LineWidth', 2);
        
        % ----- Major axis -----
        dx_major = (major/2) * cos(theta);
        dy_major = (major/2) * sin(theta);
        
        plot([x - dx_major, x + dx_major], ...
             [y - dy_major, y + dy_major], ...
             'r', 'LineWidth', 2);
        
        % ----- Minor axis -----
        dx_minor = (minor/2) * -sin(theta);
        dy_minor = (minor/2) * cos(theta);
        
        plot([x - dx_minor, x + dx_minor], ...
             [y - dy_minor, y + dy_minor], ...
             'b', 'LineWidth', 2);
        %s
        pixelIdx = stats.PixelIdxList{k};
        
        blobX = X_layer(pixelIdx);
        blobY = Y_layer(pixelIdx);
        blobZ = Z_layer(pixelIdx);

        % Filter out NaNs (invalid depth points)
        validPts = ~isnan(blobX) & ~isnan(blobY) & ~isnan(blobZ);
        if sum(validPts) > 0
            % Calculate robust 3D centroid (Median is safer than Mean for depth)
            objX = median(blobX(validPts));
            objY = median(blobY(validPts));
            objZ_cube = median(blobZ(validPts)); % This is depth from camera
        
            % 1. Get plane parameters [A, B, C, D]
            parameters = params.Parameters;
            A = parameters(1); B = parameters(2); C = parameters(3); D = parameters(4);
        
            % 2. Calculate the Z-value of the plane at the cube's (X,Y) position
            % Formula: Ax + By + Cz + D = 0  =>  z = -(Ax + By + D) / C
            objZ_plane = -(A*objX + B*objY + D) / C;
        
            % 3. Calculate Height relative to the table
            % Height is (Depth to Table) - (Depth to Cube Top)
            cubeHeight = objZ_plane - objZ_cube;
            fprintf('  Cube %s:\n', colorNames(c));
            fprintf(' [%.3f, %.3f, %.3f, %.3f] \n', A,B,C,D);
            fprintf('  Camera-Relative (X, Y, Z): [%.3f, %.3f, %.3f] m\n', objX, objY, objZ_cube);
            fprintf('  Table Depth at this point: %.3f m\n', objZ_plane);
            fprintf('  Cube Height above Table:   %.3f m (approx %.1f cm)\n\n', cubeHeight, cubeHeight*100);
        end

    end
    
    hold off
end