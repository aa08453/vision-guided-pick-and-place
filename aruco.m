 pipe = realsense.pipeline();
        profile = pipe.start();
        
        % Get camera device information
        dev = profile.get_device();
        depth_sensor = dev.first('depth_sensor');
        depth_scaling = depth_sensor.get_depth_scale();
        
        % Extract depth stream and intrinsics
        depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
        depth_intrinsics = depth_stream.get_intrinsics();
        
        % Alignment object for depth-to-color synchronization
        align_to_depth = realsense.align(realsense.stream.depth);
        
        fprintf('Camera initialized. Processing frames...\n\n');
        
        % Process frames (skip first 5 to let camera settle)
        num_frames = 5;
        for frame_idx = 1:num_frames
            % Get frames from camera
            fs = pipe.wait_for_frames();
            
            % Align color to depth
            fs = align_to_depth.process(fs);
            
            % Extract and process depth frame
            depth = fs.get_depth_frame();
            depth_data = double(depth.get_data());
            depth_frame = permute(reshape(depth_data', [depth.get_width(), depth.get_height()]), [2 1]);
            
            % Extract and process color frame
            color = fs.get_color_frame();
            color_data = color.get_data();
            color_frame = permute(reshape(color_data', [3, color.get_width(), color.get_height()]), [3 2 1]);
            
            
            % Create point cloud

          color_stream = profile.get_stream(realsense.stream.color).as('video_stream_profile');
    
         % Get and display the intrinsics
        color_intrinsics = color_stream.get_intrinsics();

            intrinsics = cameraIntrinsics([color_intrinsics.fx, color_intrinsics.fy], ...
                                          [color_intrinsics.ppx, color_intrinsics.ppy], ...
                                          size(depth_frame));
            
        end       


 T = calibrate_camera_aruco(0, -12.5, 0, 0, 'marker_size', 10, 'intrinsics', intrinsics)
