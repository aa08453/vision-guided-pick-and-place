clc; clear all; close all;

read_flag = "file" % or file 

if (read_flag == "stream")

    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    
    % Start streaming on an arbitrary camera with default settings
    profile = pipe.start();

    % Get streaming device's name
    dev = profile.get_device();  

    % Access Depth Sensor
    depth_sensor = dev.first('depth_sensor');

    % Find the mapping from 1 depth unit to meters, i.e. 1 depth unit =
    % depth_scaling meters.
    depth_scaling = depth_sensor.get_depth_scale();

    % Extract the depth stream
    depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
    
    % Get the intrinsics
    depth_intrinsics = depth_stream.get_intrinsics();

    % Get frames. We discard the first couple to allow
    % the camera time to settle
    for i = 1:5
        fs = pipe.wait_for_frames();
    end
    
    % Alignment is necessary as the depth cameras and RGB cameras are
    % physically separated. So, the same (x,y,z) in real world maps to
    % different (u,v) in the depth image and the color images. To build a
    % point cloud we only need depth image, but if we want the color the
    % cloud then we'll need the other image.

    % Since the two images are of different sizes, we can either align the
    % depth to color image, or the color to depth.
    % Change the argument to realsense.stream.color to align to the color
    % image.
    align_to_depth = realsense.align(realsense.stream.depth);
    fs = align_to_depth.process(fs);
    
    % Stop streaming
    pipe.stop();

    % Extract the depth frame
    depth = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute(reshape(depth_data',[ depth.get_width(),depth.get_height()]),[2 1]);

    % Extract the color frame
    color = fs.get_color_frame();    
    color_data = color.get_data();
    color_frame = permute(reshape(color_data',[3,color.get_width(),color.get_height()]),[3 2 1]);

    % Create a MATLAB intrinsics object
    intrinsics = cameraIntrinsics([depth_intrinsics.fx,depth_intrinsics.fy],[depth_intrinsics.ppx,depth_intrinsics.ppy],size(depth_frame));
    
    % Create a point cloud
    ptCloud = pcfromdepth(depth_frame,1/depth_scaling,intrinsics,ColorImage=color_frame);
    display(ptCloud)
elseif (read_flag == "file")
    % ptCloud = pcread("ptCloud.pcd")
    ptCloud = pcread("ptCloudMaslaExample5.pcd")
end


rgb = (ptCloud.Color);

figure;
imshow(rgb, [])
title("RGB Image")

maxDistance = 0.02; % 2 cm
referenceVector = [0, 0, 1]; % z axis
maxAngularDistance = 5; % degrees

[model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,...
            maxDistance,referenceVector,maxAngularDistance);

figure;

plane1_new = uint8(zeros(480,640, 3));
[row, col] = ind2sub([480 640], outlierIndices);
for k = 1:length(outlierIndices)
    plane1_new(row(k), col(k), :) = uint8(ptCloud.Color(row(k), col(k), :));
end

ds_factor = 1; 
plane_small = imresize(plane1_new, 1/ds_factor, 'nearest');
[rows_s, cols_s, ~] = size(plane_small);

% --- 2. Reshape to Nx3 (List of RGB pixels) ---
pixels_s = double(reshape(plane_small, [], 3));

% --- 3. Filter Background (Crucial Step) ---
% We only want to cluster pixels that aren't black (the table/background)
valid_mask = sum(pixels_s, 2) > 15; % Adjust threshold if cubes are very dark
pixelData = pixels_s(valid_mask, :);

% --- 4. Run DBSCAN on Color Data ---
epsilon = 8;    % Search radius in RGB space
minpts = 100;   % Minimum pixels to form a cluster at 1/4 resolution
idx = dbscan(pixelData, epsilon, minpts);

% --- 5. Map Labels Back to Image Shape ---
L_small = zeros(rows_s * cols_s, 1);
L_small(valid_mask) = idx;
L_small_mat = reshape(L_small, [rows_s, cols_s]);

% --- 6. Upscale to Original Resolution ---
plane1_seg = imresize(L_small_mat, [480, 640], 'nearest');

foundClusters = max(idx);
fprintf('DBSCAN found %d actual colored objects.\n', foundClusters);

numColors = 4;
numClusters = foundClusters;
imshow(plane1_new);
[plane1_seg, Centers] = imsegkmeans(plane1_new, numClusters, numAttempts=3);
L = plane1_seg;
figure;
imshow(label2rgb(plane1_seg))


dominantColors = zeros(numClusters, 3); % store mean RGB

for i = 1:numClusters
    mask = (plane1_seg == i); % pixels belonging to cluster i
    for c = 1:3
        channel = plane1_new(:,:,c);
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
    mask = (plane1_seg == i);
    
    segmented_img = zeros(size(plane1_new), 'uint8');
    for c = 1:3
        channel = plane1_new(:,:,c);
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
X_layer = ptCloud.Location(:,:,1);
Y_layer = ptCloud.Location(:,:,2);
Z_layer = ptCloud.Location(:,:,3);

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
            params = model1.Parameters;
            A = params(1); B = params(2); C = params(3); D = params(4);
        
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