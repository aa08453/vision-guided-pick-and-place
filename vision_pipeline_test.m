clc; clear all; close all;
read_flag = "file" % stream | file | test
desired = {["red"]; 
           ["red"]; 
           ["yellow"]; 
           ["red", "yellow"]; 
           ["blue"];
           ["red", "yellow", "blue", "green"]}; 


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
    imshow(ptCloud.Color)
elseif (read_flag == "test")
    % ptCloud = pcread("ptCloud.pcd")
    
    for i=1:6
        ptCloud = pcread(sprintf("ptCloudMaslaExample%i.pcd", i));
        [segmented_rgb, sorted] = vision_pipeline(ptCloud);
        keepIdx = (sorted ~= "background") & (sorted ~= "grey");
        recieved = sorted(keepIdx);

        if isequal(sort(strtrim(recieved(:))), sort(strtrim(desired{i}(:))))
            status = "✅";
        else 
            status = "❌";
        end
        rec_str = strjoin(recieved, ", ");
        des_str = strjoin(desired{i}, ", ");
        fprintf('Test %i: [Desired: %-15s] | [Recieved: %-15s] %s\n', ...
            i, des_str, rec_str, status);
    end
    
elseif (read_flag == "file")
        i = 1;
        % ptCloud = pcread(sprintf("ptCloud%i.ply", i));
        % newLocation = reshape(ptCloud.Location, [480, 640, 3]);
        % newColor = reshape(ptCloud.Color, [480, 640, 3]);
        % ptCloud = pointCloud(newLocation, 'Color', newColor);
        
        ptCloud = pcread(sprintf("ptCloudMaslaExample%i.pcd", i));
        
        [segmented_rgb, sorted] = vision_pipeline(ptCloud);
        keepIdx = (sorted ~= "background") & (sorted ~= "grey");
        recieved = sorted(keepIdx);

        if isequal(sort(strtrim(recieved(:))), sort(strtrim(desired{i}(:))))
            status = "✅";
        else 
            status = "❌";
        end
        rec_str = strjoin(recieved, ", ");
        des_str = strjoin(desired{i}, ", ");
        fprintf('Test %i: [Desired: %-15s] | [Recieved: %-15s] %s\n', ...
            i, des_str, rec_str, status);

end
