known = struct('color', {}, 'robot', {});
known(1) = struct('color',"yellow", 'robot',[-20.25, 0, 2.5]);
known(2) = struct('color',"red",    'robot',[0,  19.5, 2.5]);
known(3) = struct('color',"blue",  'robot',[20.5, 0, 2.5]);
known(4) = struct('color',"green",   'robot',[-20.25, 0, 2.5]);


% Initialize RealSense pipeline
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
            
            % Create camera intrinsics object
            intrinsics = cameraIntrinsics([depth_intrinsics.fx, depth_intrinsics.fy], ...
                                          [depth_intrinsics.ppx, depth_intrinsics.ppy], ...
                                          size(depth_frame));
            
            % Create point cloud
        end

ptCloud = pcfromdepth(depth_frame, 1/depth_scaling, intrinsics, ColorImage=color_frame);
save_path = "C:\Users\itadmin\Desktop\vision-guided-pick-and-place";
calibrate_camera(ptCloud, known, save_path);