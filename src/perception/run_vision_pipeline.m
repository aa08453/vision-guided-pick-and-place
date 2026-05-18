function run_vision_pipeline(mode, desired)
    
    % Default to 'test' if no mode specified
    if nargin == 0
        mode = "test";
    end
    
    % Validate mode input
    mode = string(mode);
    valid_modes = ["stream", "file", "test"];
    if ~ismember(mode, valid_modes)
        error('Invalid mode. Must be one of: %s', strjoin(valid_modes, ', '));
    end
    

    % Route to appropriate mode handler
    switch mode
        case "stream"
            fprintf('\n========== STREAM MODE ==========\n');
    fprintf('Starting RealSense camera stream...\n\n');
    
    try
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
        
        fprintf('Frame %d/%d: Received point cloud with %d points\n', ...
            frame_idx, num_frames, ptCloud.Count);
        
        % Process through vision pipeline
        [segmented_rgb, sorted] = vision_pipeline(ptCloud);
        
        % Validate results against desired output
        validate_results(sorted, desired, frame_idx);
        
        % Cleanup
        pipe.stop();
        fprintf('\n========== STREAM MODE COMPLETE ==========\n\n');
        
    catch ME
        fprintf('Error in stream mode: %s\n', ME.message);
        if exist('pipe', 'var')
            pipe.stop();
        end
        rethrow(ME);
    end
        case "file"
             fprintf('\n========== FILE MODE ==========\n');
    
    file_idx = 1;
    filename = sprintf("ptCloudMaslaExample%i.pcd", file_idx);
    
    if ~isfile(filename)
        error('File not found: %s\nMake sure the file is in the current directory.', filename);
    end
    
    fprintf('Processing file: %s\n\n', filename);

        desired = {["red"]; 
               ["red"]; 
               ["yellow"]; 
               ["red", "yellow"]; 
               ["blue"];
               ["red", "yellow", "blue", "green"]};
    
    
    try
        % Load point cloud from file
        ptCloud = pcread(filename);
        fprintf('Loaded point cloud with %d points\n\n', ptCloud.Count);
        
        % Process through vision pipeline
        [segmented_rgb, sorted] = vision_pipeline(ptCloud);
        
        % Validate results
        fprintf('\nValidation Results:\n');
        fprintf('-------------------\n');
        validate_results(sorted, desired{file_idx}, file_idx);
        
    catch ME
        fprintf('Error in file mode: %s\n', ME.message);
        rethrow(ME);
    end
    
    fprintf('\n========== FILE MODE COMPLETE ==========\n\n');
        case "test"
            run_test_mode(desired);
        otherwise
            error('Unknown mode: %s', mode);
    end
end








    

