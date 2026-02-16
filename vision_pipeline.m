function [segmented_rgb, sortedLabels] = vision_pipeline(pointCloud)
[sliced_rgb, params] = find_plane(pointCloud);
[segmented_rgb, Centers, numClusters] = segment(sliced_rgb); % Gives segment mask
imshow(label2rgb(segmented_rgb));
[mergedSegments, sortedLabels] = mergeSegments(sliced_rgb, segmented_rgb);


% Map 1->Yellow, 2->Red, 3->Green, 4->Blue
customColormap = [1 1 0;  % 1: Yellow
                  1 0 0;  % 2: Red
                  0 1 0;  % 3: Green
                  0 0 1]; % 4: Blue

rgbLabelImage = label2rgb(mergedSegments, customColormap, 'k'); % 'k' for black background
imshow(rgbLabelImage);
title('Collapsed 2D Label Map');


%s
X_layer = pointCloud.Location(:,:,1);
Y_layer = pointCloud.Location(:,:,2);
Z_layer = pointCloud.Location(:,:,3);



% figure;
% 
% colorNames = ["Red","Green","Blue","Yellow"];
% for c = 1:4
% 
%     binaryMask = segmented_colors(:,:,c) > 0;
% 
%     %s
%     % 1. Fill small holes inside the cube
%     binaryMask = imfill(binaryMask, 'holes');
%     % 2. Remove tiny noise (anything smaller than 500 pixels)
%     binaryMask = bwareaopen(binaryMask, 500); 
%     % 3. Bridge small gaps between parts of the same cube
%     se = strel('disk', 5);
%     binaryMask = imclose(binaryMask, se);
% 
%     CC = bwconncomp(binaryMask);
% 
%     stats = regionprops("table", CC, ...
%         "Area", "Centroid", "MajorAxisLength", ...
%         "MinorAxisLength", "Orientation", "PixelIdxList"); %s
% 
%     %s
%     if isempty(stats)
%         fprintf('No object found for color: %s\n', colorNames(c));
%         continue; 
%     end
% 
%     %s
%     % 1 co-ordinate per color
%     [~, largestIdx] = max(stats.Area);
%     pixelIdx = stats.PixelIdxList{largestIdx};
% 
%     subplot(2,2,c)
%     imshow(binaryMask)
%     title(colorNames(c))
%     hold on
% 
%     for k = 1:height(stats)
% 
%         % Extract region properties
%         x = stats.Centroid(k,1);
%         y = stats.Centroid(k,2);
%         major = stats.MajorAxisLength(k);
%         minor = stats.MinorAxisLength(k);
%         theta = deg2rad(-stats.Orientation(k));
% 
%         % ----- Centroid -----
%         plot(x, y, 'go', 'MarkerSize', 6, 'LineWidth', 2);
% 
%         % ----- Major axis -----
%         dx_major = (major/2) * cos(theta);
%         dy_major = (major/2) * sin(theta);
% 
%         plot([x - dx_major, x + dx_major], ...
%              [y - dy_major, y + dy_major], ...
%              'r', 'LineWidth', 2);
% 
%         % ----- Minor axis -----
%         dx_minor = (minor/2) * -sin(theta);
%         dy_minor = (minor/2) * cos(theta);
% 
%         plot([x - dx_minor, x + dx_minor], ...
%              [y - dy_minor, y + dy_minor], ...
%              'b', 'LineWidth', 2);
%         %s
%         pixelIdx = stats.PixelIdxList{k};
% 
%         blobX = X_layer(pixelIdx);
%         blobY = Y_layer(pixelIdx);
%         blobZ = Z_layer(pixelIdx);
% 
%         % Filter out NaNs (invalid depth points)
%         validPts = ~isnan(blobX) & ~isnan(blobY) & ~isnan(blobZ);
%         if sum(validPts) > 0
%             % Calculate robust 3D centroid (Median is safer than Mean for depth)
%             objX = median(blobX(validPts));
%             objY = median(blobY(validPts));
%             objZ_cube = median(blobZ(validPts)); % This is depth from camera
% 
%             % 1. Get plane parameters [A, B, C, D]
%             parameters = params.Parameters;
%             A = parameters(1); B = parameters(2); C = parameters(3); D = parameters(4);
% 
%             % 2. Calculate the Z-value of the plane at the cube's (X,Y) position
%             % Formula: Ax + By + Cz + D = 0  =>  z = -(Ax + By + D) / C
%             objZ_plane = -(A*objX + B*objY + D) / C;
% 
%             % 3. Calculate Height relative to the table
%             % Height is (Depth to Table) - (Depth to Cube Top)
%             cubeHeight = objZ_plane - objZ_cube;
%             fprintf('  Cube %s:\n', colorNames(c));
%             fprintf(' [%.3f, %.3f, %.3f, %.3f] \n', A,B,C,D);
%             fprintf('  Camera-Relative (X, Y, Z): [%.3f, %.3f, %.3f] m\n', objX, objY, objZ_cube);
%             fprintf('  Table Depth at this point: %.3f m\n', objZ_plane);
%             fprintf('  Cube Height above Table:   %.3f m (approx %.1f cm)\n\n', cubeHeight, cubeHeight*100);
%         end
% 
%     end
% 
%     hold off
end