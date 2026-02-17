ptCloud = pcread("ptCloudMaslaExample1.pcd")

rgb = (ptCloud.Color);

maxDistance = 0.02; % 2 cm
referenceVector = [0, 0, 1]; % z axis
maxAngularDistance = 5; % degrees

[model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,...
            maxDistance,referenceVector,maxAngularDistance);

maxDistance = 0.02; % 2 cm
referenceVector = [0, 0, 1]; % z axis
maxAngularDistance = 5; % degrees

[model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,...
            maxDistance,referenceVector,maxAngularDistance);

plane1_new = uint8(zeros(480,640, 3));
[row, col] = ind2sub([480 640], outlierIndices);
for k = 1:length(outlierIndices)
    plane1_new(row(k), col(k), :) = single(ptCloud.Color(row(k), col(k), :));
end



% --- 1. Downsample for Speed ---
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

imshow(plane1_seg,[])