clc; clear all; close all;

read_flag = "stream" % or file 

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
    ptCloud = pcfromdepth(depth_frame,1/depth_scaling,intrinsics,ColorImage=color_frame)
elseif (read_flag == "file")
    ptCloud = pcread("ptCloud.pcd");
end
rgb = (ptCloud.Color);

figure;
imshow(rgb, [])
title("RGB Image")


% pcshow(pointCloud(lightpoints), "VerticalAxisDir","Down");

pixel_labels = segment_cluster(rgb, 3);
% overlay = labeloverlay(rgb, pixel_labels);
display = label2rgb(pixel_labels, 'jet');
% imshow(display);


mask = (pixel_labels == 3);


X = ptCloud.Location(:,:,1);
Y = ptCloud.Location(:,:,2);
Z = ptCloud.Location(:,:,3);

points_xyz = [ ...
    X(mask), ...
    Y(mask), ...
    Z(mask) ...
];



% imshow(mask, []);

figure;
pcshow(ptCloud, "VerticalAxisDir", "Down");
title("Original Point Cloud")


maxDistance = 0.02; % 0.3 cm
referenceVector = [0, 0, 1]; % z axis
maxAngularDistance = 5; % degrees

[model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,...
            maxDistance,referenceVector,maxAngularDistance);




figure;

ptCloud_hsv = single(rgb2hsv(ptCloud.Color));

% Sliced using a plane
plane1_new = (zeros(480,640, 3));
[row, col] = ind2sub([480 640], outlierIndices);
for k = 1:length(outlierIndices)
    plane1_new(row(k), col(k), :) = single(ptCloud_hsv(row(k), col(k), :));
end

% Calculated a number of clusters
numClusters = 6;
imshow(plane1_new);
[plane1_seg, Centers] = imsegkmeans(plane1_new, numClusters, numAttempts=3);
L = plane1_seg;
figure;
imshow(label2rgb(plane1_seg))

range = 10;
y = [135 51 0];
r = [87 13 17];
g = [1 37 5];
b = [0 54 68];
background = [0 0 0];

iterations = double(rgb2hsv([y; r; g; b; background;]));
my_labels = ['y','r', 'g', 'b', 'background'];

numLabels = length(my_labels);
Centers = double(Centers);

for i = 1:size(Centers,1)

    losses = vecnorm(iterations - Centers(i,:), 2, 2);

    [~, idx] = min(losses);

    fprintf("Cluster %d is %s\n", i, my_labels(idx));
end


figure;

numClusters = size(Centers,1);

for i = 1:numClusters

    losses = vecnorm(iterations - Centers(i,:), 2, 2);
    [~, idx] = min(losses);

    identified_color = string(my_labels(idx));

    mask = (plane1_seg == i);
    segmented_img = zeros(size(plane1_new), 'uint8');
    for c = 1:3
        channel = plane1_new(:,:,c);
        channel(~mask) = 0;
        segmented_img(:,:,c) = channel;
    end

    subplot(2,3,i)
    imshow(segmented_img)
    title(sprintf("Cluster %d → %s", i, identified_color))

end


