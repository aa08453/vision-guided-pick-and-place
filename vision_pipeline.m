clc; clear all; close all;
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
    
rgb = (ptCloud.Color);

figure;
imshow(rgb, [])
title("RGB Image")


% Lab = rgb2lab(rgb);
% figure;
% histogram(Lab(:,:,2)); %a
% title("Lab a histogram")
% xlabel("a");
% ylabel("freq");
% axis tight;
% 
% imshow(Lab(:,:,2),[]);
% title("Lab a space")
% 
% histogram(Lab(:,:,3)); %b
% title("Lab b histogram")
% xlabel("b");
% ylabel("freq");
% axis tight;
% 
% imshow(Lab(:,:,3), []);
% title("Lab a histogram")
% 
% lightMask = (Lab(:,:,2) < -2);
% imshow(lightMask, []);
% title("Lab b space with masking")


% bwMask = bwareaopen(bwMask, 100);
% bwMask = imfill(bw2, 'holes');
% 
% X = ptCloud.Location(:,:,1);
% Y = ptCloud.Location(:,:,2);
% Z = ptCloud.Location(:,:,3);
% 
% lightpoints = [X(), Y(), Z()];


% pcshow(pointCloud(lightpoints), "VerticalAxisDir","Down");

% pixel_labels = segment_cluster(rgb, 3);
% overlay = labeloverlay(rgb, pixel_labels);
% display = label2rgb(pixel_labels, 'jet');
% imshow(display);


% mask = (pixel_labels == 3);
% 
% X = ptCloud.Location(:,:,1);
% Y = ptCloud.Location(:,:,2);
% Z = ptCloud.Location(:,:,3);
% 
% points_xyz = [ ...
%     X(mask), ...
%     Y(mask), ...
%     Z(mask) ...
% ];



% imshow(mask, []);

figure;
pcshow(ptCloud, "VerticalAxisDir", "Down");
pcwrite(ptCloud,'ptCloudMaslaExample5.pcd','Encoding','ascii');
title("Original Point Cloud")

% % 
% figure;
% pc = pointCloud(points_xyz);
% pcshow(pc, "VerticalAxisDir","Down");

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

% remainPtCloud = select(ptCloud,outlierIndices);

% figure;
% pcshow(remainPtCloud, "VerticalAxisDir", "Down")
% title("Cubes without Plane")
numColors = 4;
numClusters = 6;
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



%==============================================================
% for label_i = 1:numClusters
%     losses = zeros(1,numLabels);
%     for label_known = 1:numLabels
%         potential_label = iterations(label_known,:);
%         % Need to pull the whole mask 
%         segmented_mask = (plane1_seg(:,:) == label_known);
%         plane1_masked = uint8(segmented_mask) .* uint8(plane1_new);
%         losses(1,label_known) = mean(sqrt(plane1_masked.^2 - potential_label.^2));
%     end
%     [min, idx] = min(losses);
%     color = my_labels(idx);
%     display(color)
% end
% 

% for label_i = 1:numClusters
%     % Get mask of this cluster
%     segmented_mask = (plane1_seg == label_i);
%     % Extract RGB pixels of that cluster
%     pixels = plane1_new(repmat(segmented_mask,[1 1 3]));
%     pixels = reshape(pixels, [], 3);
%     % Convert to double for math
%     pixels = double(pixels);
%     % Compute mean color of cluster
%     mean_color = mean(pixels, 1);
%     losses = zeros(1, size(iterations,1));
%     % Compare against known colors
%     for label_known = 1:size(iterations,1)
%         ref = iterations(label_known,:);
%         % Euclidean distance
%         losses(label_known) = norm(mean_color - ref);
%     end
%     [~, idx] = min(losses);
%     fprintf("Cluster %d is %s\n", label_i, my_labels(idx));
% 
% end








% impixelinfo(plane1_new)

% mask_re;
% mask_yellow = (H > 0.12 & H < 0.18) & S > 0.4 & V > 0.3;
% mask_green = (H > 0.25 & H < 0.45) & S > 0.4 & V > 0.3;
% mask_blue = (H > 0.55 & H < 0.75) & S > 0.4 & V > 0.3;
% 
% mask_red    = bwareaopen(mask_red, 200);
% mask_green  = bwareaopen(mask_green, 200);
% mask_blue   = bwareaopen(mask_blue, 200);
% mask_yellow = bwareaopen(mask_yellow, 200);


% 
% regionLabels = plane1_seg(mask_red);
% dominantCluster = mode(regionLabels);


% plane1 = select(ptCloud,inlierIndices);
% pcshow(plane1, "VerticalAxisDir", "Down")
% title("Workspace plane")





% 
% imshow(Lab(:,:,3), []);
% title("Lab a histogram")
% 
% lightMask = (Lab(:,:,2) < -2);
% imshow(lightMask, []);
% title("Lab b space with masking")


% imshow(rgb_cubes, []);
% 
% planeModel = pcfitplane(ptCloud, 0.01);
% inlierIdx = planeModel.InlierIndices;
% 
% nonPlaneMask = true(m*n,1);
% nonPlaneMask(inlierIdx) = false;
% nonPlaneMask = reshape(nonPlaneMask, m, n);

% validMask = validMask & nonPlaneMask;











% X = ptCloud.Location(:,:,1);
% Y = ptCloud.Location(:,:,2);
% Z = ptCloud.Location(:,:,3);
% 
% validMask = Z > 0 & ~isnan(Z); % getting valid depth values
% 
% rgb_double = im2double(rgb);
% 
% [m,n,~] = size(rgb_double); % ignore channels
% pixels = reshape(rgb_double, m*n, 3);
% 
% % getting rgb values corresponding to correct Z
% valid_pixels = pixels(validMask(:), :);
% 
% [idx_valid, C] = kmeans(valid_pixels, 7, 'Replicates', 3); % replicate = parallelize
% 
% pixel_labels(validMask(:)) = idx_valid;
% 
% pixel_labels = reshape(pixel_labels, m, n);
% 
% 
% 
% display = label2rgb(pixel_labels, 'jet');
% imshow(display);
% title("RGB Clustering")
% 
% 
% mask = (pixel_labels == greenCluster);
% 
% points_xyz = [ ...
%     X(mask), ...
%     Y(mask), ...
%     Z(mask) ...
% ];
% pc = pointCloud(points_xyz);
% pcshow(pc, "VerticalAxisDir","Down");
% title("RGB Masked Point Cloud")



% hsv_img = rgb2hsv(rgb_double);
% pixels = reshape(hsv_img, m*n, 3);
% valid_pixels = pixels(validMask(:), 1:2);  % Use Hue + Saturation only
% 
% % [idx_valid, C] = kmeans(valid_pixels, 4, 'Replicates', 3); % replicate = parallelize
% pixel_labels = segment_cluster(image_rgb, numColors, debug)
% 
% pixel_labels = zeros(m*n,1);
% pixel_labels(validMask(:)) = idx_valid;
% 
% pixel_labels = reshape(pixel_labels, m, n);
% 
% display = label2rgb(pixel_labels, 'jet');
% imshow(display);
% title("HSV Clustering")


