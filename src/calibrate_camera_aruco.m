function T = calibrate_camera_aruco(x, y, z, yaw_deg, varargin)
    % CALIBRATE_CAMERA_ARUCO  One-shot extrinsic calibration via a single
    % ArUco marker.
    %
    %   T = calibrate_camera_aruco(x, y, z, yaw_deg)
    %   T = calibrate_camera_aruco(x, y, z, yaw_deg, Name, Value, ...)
    %
    % Procedure:
    %   1. Lay your 4x4 ArUco marker (ID 0, 10 cm) flat on the table.
    %   2. Place its CENTER at robot coordinates (x, y, z) in cm.
    %      For a marker on the table surface, z = 0.
    %   3. Rotate the marker so its TOP edge (the side with the upright
    %      "look" of the pattern) points along robot +X. yaw_deg = 0.
    %      If the marker is rotated, set yaw_deg to the rotation about
    %      robot +Z in degrees (right-hand rule, CCW positive when viewed
    %      from above).
    %   4. Call this function. It opens the RealSense, grabs one frame,
    %      detects the marker, solves PnP, composes T, and saves to disk.
    %
    % Optional name-value pairs:
    %   'family'        ArUco dictionary. Default 'DICT_4X4_250'.
    %                   For ID=0 every DICT_4X4_* dictionary works.
    %                   If detection fails, try 'DICT_4X4_50' or
    %                   'DICT_4X4_100'.
    %   'marker_size'   Side length in cm. Default 10 (your marker).
    %   'save_path'     Output .mat path. Default 'calib.mat'. ''=don't save.
    %   'image'         Pre-captured RGB image (skips RealSense).
    %   'intrinsics'    cameraIntrinsics object (required if 'image' given).
    %
    % Returns:
    %   T  4x4 transform such that  p_robot_cm = T * [p_cam_cm; 1].
    %      Drop-in compatible with cam_to_robot.m and the existing
    %      vision_pick_and_place flow.

    p = inputParser;
    p.addParameter('family',      'DICT_4X4_250', @(s) ischar(s) || isstring(s));
    p.addParameter('marker_size', 10.0,           @isnumeric);     % cm
    p.addParameter('save_path',   'calib.mat',    @(s) ischar(s) || isstring(s));
    p.addParameter('image',       [],             @(v) isempty(v) || (isnumeric(v) && ndims(v) == 3));
    p.addParameter('intrinsics',  [],             @(v) isempty(v) || isa(v, 'cameraIntrinsics'));
    p.parse(varargin{:});
    opts = p.Results;

    % ---- 1. Acquire image + color intrinsics --------------------------------
    if isempty(opts.image)
        fprintf('Opening RealSense at 640x480...\n');
        [I, color_intr] = local_capture_color_rs();
    else
        I = opts.image;
        color_intr = opts.intrinsics;
        if isempty(color_intr)
            error('When passing an image, also pass ''intrinsics''.');
        end
    end

    % ---- 2. Detect the marker -----------------------------------------------
    [ids, locs] = readArucoMarker(I, opts.family);
    if isempty(ids)
        figure; imshow(I); title('No ArUco marker detected');
        error(['No %s markers detected. Check that the marker is visible, ' ...
               'well-lit, and not glaring. Try a different ''family''.'], ...
               opts.family);
    end
    fprintf('Detected %d marker(s): IDs = %s\n', numel(ids), mat2str(ids));

    % Use ID 0 if present, otherwise the first detection
    idx = find(ids == 0, 1);
    if isempty(idx), idx = 1; end
    fprintf('Using marker ID = %d\n', ids(idx));
    corners_2d = locs(:, :, idx);    % 4x2

    % ---- 3. Marker corners in marker-local frame (cm) -----------------------
    % MATLAB returns corners CCW starting from top-left, which matches the
    % OpenCV-standard ArUco order (TL, TR, BR, BL) when viewing the marker
    % face-on.
    s = opts.marker_size;
    % estimateExtrinsics is a PLANAR PnP solver -- it expects world points
    % with only X and Y (z is assumed 0). The returned extrinsics still
    % give the full 3D marker-frame -> camera-frame transform.
    corners_2d_marker = [
        -s/2,  s/2;   % TL
         s/2,  s/2;   % TR
         s/2, -s/2;   % BR
        -s/2, -s/2    % BL
    ];

    % ---- 4. PnP: marker frame -> camera frame -------------------------------
    ext = estimateExtrinsics(corners_2d, corners_2d_marker, color_intr);
    T_marker_to_cam = [ext.R, ext.Translation(:); 0 0 0 1]
    

    % ---- 5. Marker pose in robot frame (from user's args) -------------------
    th = deg2rad(yaw_deg);
    R_mr = [cos(th), -sin(th), 0;
            sin(th),  cos(th), 0;
            0,        0,       1];
    T_marker_to_rob = [R_mr, [x; y; z]; 0 0 0 1];

    % ---- 6. Compose ---------------------------------------------------------
    T_cam_to_marker = inv(T_marker_to_cam);
    T = T_marker_to_rob * T_cam_to_marker;

    % ---- 7. Reprojection residuals (sanity check) ---------------------------
    % worldToImage wants 3D points -- add a z=0 column back in.
    corners_3d_marker = [corners_2d_marker, zeros(4, 1)];
    proj = worldToImage(color_intr, ext, corners_3d_marker);
    residuals = vecnorm(proj - corners_2d, 2, 2);
    labels = {'TL', 'TR', 'BR', 'BL'};
    fprintf('\nReprojection residuals (pixels):\n');
    for k = 1:4
        fprintf('  %s: %.2f px\n', labels{k}, residuals(k));
    end
    fprintf('  RMS:   %.2f px\n', sqrt(mean(residuals.^2)));
    if max(residuals) > 5
        warning(['Reprojection error > 5 px. Calibration may be inaccurate.\n' ...
                 'Check: (1) marker_size matches printed size exactly, ' ...
                 '(2) marker is flat (not curling/warped), ' ...
                 '(3) marker is not heavily tilted in the image.']);
    end

    % ---- 8. Visual verification ---------------------------------------------
    figure('Name', 'ArUco detection');
    imshow(I); hold on;
    % Draw the detected quad
    quad = corners_2d([1 2 3 4 1], :);
    plot(quad(:,1), quad(:,2), 'g-', 'LineWidth', 2);
    for k = 1:4
        plot(corners_2d(k,1), corners_2d(k,2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
        text(corners_2d(k,1)+8, corners_2d(k,2), labels{k}, ...
            'Color', 'y', 'FontSize', 12, 'FontWeight', 'bold');
    end
    title(sprintf('Marker ID=%d  size=%.1f cm  RMS=%.2f px', ...
        ids(idx), s, sqrt(mean(residuals.^2))));
    hold off;

    % ---- 9. Print transform and sanity-check the camera position -----------
    fprintf('\nT_cam_to_robot =\n');
    disp(T);

    cam_origin_in_robot = T(1:3, 4)';
    fprintf('Predicted camera origin in robot frame: (%.2f, %.2f, %.2f) cm\n', ...
        cam_origin_in_robot);
    fprintf(['(Should match where the lens physically sits relative to ' ...
             'the robot base. Sanity-check with a ruler.)\n']);

    % ---- 10. Save -----------------------------------------------------------
    if ~isempty(opts.save_path)
        save(opts.save_path, 'T');
        fprintf('\nSaved transform to %s\n', opts.save_path);
    end
end


% =========================================================================
% Helpers
% =========================================================================
function [I, color_intr] = local_capture_color_rs()
    pipe = realsense.pipeline();
    cfg  = realsense.config();
    cfg.enable_stream(realsense.stream.depth, 640, 480, realsense.format.z16, 30);
    cfg.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30);
    profile = pipe.start(cfg);
    cleanup = onCleanup(@() pipe.stop()); %#ok<NASGU>

    fprintf('Warming up...\n');
    for i = 1:10, pipe.wait_for_frames(); end

    align_to_color = realsense.align(realsense.stream.color);
    fs = pipe.wait_for_frames();
    fs = align_to_color.process(fs);

    color = fs.get_color_frame();
    color_data = color.get_data();
    I = permute(reshape(color_data', ...
        [3, color.get_width(), color.get_height()]), [3 2 1]);

    color_stream = profile.get_stream(realsense.stream.color) ...
        .as('video_stream_profile');
    ci = color_stream.get_intrinsics();
    color_intr = cameraIntrinsics([ci.fx, ci.fy], [ci.ppx, ci.ppy], ...
                                  [size(I,1), size(I,2)]);
end
