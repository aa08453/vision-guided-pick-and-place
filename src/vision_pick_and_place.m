function vision_pick_and_place(arm, varargin)
    % VISION_PICK_AND_PLACE  Top-level orchestrator: snapshot -> detect -> pick.
    %
    %   vision_pick_and_place(arm)
    %   vision_pick_and_place(arm, 'Name', Value, ...)
    %
    % Workflow per loop iteration:
    %   1. Capture a workspace snapshot (RealSense or saved .ply).
    %   2. snapshot_cubes() returns a struct array of {color, x, y, z, ...}
    %      in robot frame.
    %   3. Print the list, prompt the user:
    %        <color>      pick the first cube of that color (closest to base)
    %        all <color>  pick every cube of that color, placing in the tray
    %        snapshot     re-snapshot (cubes were moved between picks)
    %        quit         exit cleanly
    %   4. All picks are top-down (phi = -pi/2). Gripper closes to
    %      cube_minor_axis - overshoot for a firm clamp.
    %   5. Placements go to a fixed tray location (with a small vertical
    %      offset per cube so batch picks stack rather than collide).
    %
    % Required arguments:
    %   arm                  Arm object, created externally with Arm('sim')
    %                        or Arm('real', 'COMx').
    %
    % Name-Value options:
    %   'source'             'realsense' (default) or 'file'
    %   'filepath'           path to .ply, used when source='file'
    %   'calibration_file'   default 'camera_transform.mat' (in pwd)
    %   'tray_pos'           1x3 cm, placement target (default [20 -15 5])
    %   'approach_dist'      cm above cube for pregrasp (default 7)
    %   'grip_overshoot'     cm subtracted from cube minor axis when clamping
    %                        (default 0.7). Higher = firmer grip.
    %   'home_after_each'    bool, return to (0,0,25,NaN) between picks (default true)
    %   'show_debug'         bool, show segmentation figures (default false)
    %
    % Example:
    %   arm = Arm('real', 'COM4');
    %   vision_pick_and_place(arm, 'tray_pos', [22 -12 4]);

    % ---- Parse options ------------------------------------------------------
    p = inputParser;
    p.addParameter('source',           'realsense',           @(s)ischar(s)||isstring(s));
    p.addParameter('filepath',         '',                    @(s)ischar(s)||isstring(s));
    p.addParameter('calibration_file', 'calib.mat',@(s)ischar(s)||isstring(s));
    p.addParameter('tray_pos',         [20, -15, 5],          @(v)isnumeric(v)&&numel(v)==3);
    p.addParameter('approach_dist',    7,                     @isnumeric);
    p.addParameter('grip_overshoot',   0.7,                   @isnumeric);
    p.addParameter('cube_z',           2.5,                   @isnumeric);
                                                              %     ^ cube top
                                                              %       height (cm).
    p.addParameter('pause_dur',        1.0,                   @isnumeric);
                                                              %     ^ seconds
                                                              %       between
                                                              %       each
                                                              %       movement
                                                              %       segment.
    p.addParameter('home_after_each',  true,                  @islogical);
    p.addParameter('show_debug',       false,                 @islogical);
    p.parse(varargin{:});
    opts = p.Results;

    % ---- Load calibration ---------------------------------------------------
    if isfile(opts.calibration_file)
        S = load(opts.calibration_file);
        if ~isfield(S, 'T')
            error('vision_pick_and_place: %s does not contain variable T', opts.calibration_file);
        end
        % S.T(1, 4) = S.T(1,4) + 2;
        % S.T(2,4) = S.T(2,4) + 2;
        T_cam_to_rob = S.T;
        fprintf('Loaded camera->robot transform from %s\n', opts.calibration_file);
    else
        warning('vision_pick_and_place:noCal', ...
            ['Calibration file %s not found. Using identity transform -- ' ...
             'picks WILL go to wrong coordinates until you run calibrate_camera.'], ...
            opts.calibration_file);
        T_cam_to_rob = [ 0,   -1,   0,  -3;
                         -1,   0,   0,  -3;
                          0,    0,  -1,   59;
                          0,         0,         0,    1.0000;
        ];

    end



    % ---- Camera pipeline lifecycle -----------------------------------------
    pipe = [];
    cleanup = onCleanup(@() local_cleanup(pipe));

    if strcmp(opts.source, 'realsense')
        pipe = realsense.pipeline();
        profile = pipe.start();
        cfg = realsense.config();
        % Force 640x480 so the HSV pipeline's pixel area thresholds and the
        % 480x480 crop stay calibrated. Without an explicit config, the
        % default stream resolution may be 1080x1920, which blows past every
        % area cutoff and breaks find_plane's indexing.
        cfg.enable_stream(realsense.stream.depth, 640, 480, realsense.format.z16, 30);
        cfg.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30);
        % pipe.start(cfg);
        fprintf('RealSense pipeline started at 640x480. Warming up...\n');
        for i = 1:10, pipe.wait_for_frames(); end
    end

    % ---- Interactive loop ---------------------------------------------------
    place_counter = 0;
    while true
        % --- Snapshot
        try
            ptCloud = local_capture(opts, profile, pipe);
            pcshow(ptCloud, "VerticalAxisDir", "Down");
        catch ME
            fprintf('Snapshot failed: %s\n', ME.message);
            break;
        end

        cubes = snapshot_cubes(ptCloud, T_cam_to_rob, ...
                               struct('show_debug', opts.show_debug));

        % --- Report
        local_print_cubes(cubes);

        % --- Prompt
        cmd = strtrim(input('\n> Color | all <color> | snapshot | quit : ', 's'));
        if isempty(cmd), continue; end
        cmd_lower = lower(cmd);

        if any(strcmp(cmd_lower, ["quit","q","exit"]))
            fprintf('Exiting.\n');
            break;

        elseif any(strcmp(cmd_lower, ["snapshot","s","scan"]))
            continue;

        elseif startsWith(cmd_lower, "all ")
            color = strtrim(extractAfter(cmd_lower, "all "));
            matches = cubes(arrayfun(@(c) c.color == color, cubes));
            if isempty(matches)
                fprintf('No %s cubes detected.\n', color);
                continue;
            end
            fprintf('Picking %d %s cube(s)...\n', numel(matches), color);
            for i = 1:numel(matches)
                place_counter = place_counter + 1;
                local_pick_one(arm, matches(i), opts, place_counter);
                if opts.home_after_each, local_go_home(arm); end
            end

        else
            color = cmd_lower;
            matches = cubes(arrayfun(@(c) c.color == color, cubes));
            if isempty(matches)
                fprintf('No %s cubes detected. Try again.\n', color);
                continue;
            end
            % Pick the cube closest to the robot base (smallest XY radius)
            radii = arrayfun(@(c) hypot(c.pos_robot(1), c.pos_robot(2)), matches);
            [~, idx] = min(radii);
            place_counter = place_counter + 1;
            local_pick_one(arm, matches(idx), opts, place_counter);
            if opts.home_after_each, local_go_home(arm); end
        end
    end
end


% =========================================================================
% Helpers
% =========================================================================
function ptCloud = local_capture(opts, profile, pipe)
    if strcmp(opts.source, 'file')
        if isempty(opts.filepath) || ~isfile(opts.filepath)
            error('source=''file'' requires a valid ''filepath''.');
        end
        ptCloud = pcread(opts.filepath);
    else
        % RealSense organized cloud via pcfromdepth + intrinsics

        depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
        depth_intrinsics = depth_stream.get_intrinsics();
        align_to_depth = realsense.align(realsense.stream.depth);


 
        fs = pipe.wait_for_frames();
        fs = align_to_depth.process(fs);

            dev = profile.get_device();  

    % Access Depth Sensor
    depth_sensor = dev.first('depth_sensor');

    % Find the mapping from 1 depth unit to meters, i.e. 1 depth unit =
    % depth_scaling meters.
    depth_scaling = depth_sensor.get_depth_scale();

        depth = fs.get_depth_frame();
        depth_data = double(depth.get_data());
        depth_frame = permute(reshape(depth_data',[ depth.get_width(),depth.get_height()]),[2 1]);
        % depth_scaling = depth_sensor.get_depth_scale();

        % Extract the color frame
        color = fs.get_color_frame();    
        color_data = color.get_data();
        color_frame = permute(reshape(color_data',[3,color.get_width(),color.get_height()]),[3 2 1]);

        intrinsics = cameraIntrinsics([depth_intrinsics.fx,depth_intrinsics.fy],[depth_intrinsics.ppx,depth_intrinsics.ppy],size(depth_frame));
    
        % Create a point cloud
        ptCloud = pcfromdepth(depth_frame,1/depth_scaling,intrinsics,ColorImage=color_frame);


    end
end

function local_cleanup(pipe)
    if ~isempty(pipe)
        try, pipe.stop(); catch, end
        fprintf('RealSense pipeline stopped.\n');
    end
end

function local_print_cubes(cubes)
    if isempty(cubes)
        fprintf('\n[snapshot] no cubes detected.\n');
        return;
    end
    fprintf('\n[snapshot] %d cube(s) detected:\n', numel(cubes));
    % Color logic disabled -- show index in place of color label. The
    % .color field is still populated (always "any") so the original
    % column-formatted print below stays runnable if color is re-enabled.
    fprintf('  %-4s  %-21s  %-8s  %-12s  %-5s\n', ...
        '#', 'pos_robot (x,y,z) cm', 'yaw deg', 'maj x min cm', 'h cm');
    for i = 1:numel(cubes)
        c = cubes(i);
        fprintf('  %-4d  (%5.1f,%5.1f,%5.1f)  %7.1f   %4.1f x %4.1f    %4.1f\n', ...
            i, c.pos_robot, rad2deg(c.yaw_robot), c.major_cm, c.minor_cm, c.height_cm);
    end

    % ---- ORIGINAL COLOR-AWARE TABLE (disabled) -----------------------------
    % fprintf('  %-7s  %-21s  %-8s  %-12s  %-5s\n', ...
    %     'color', 'pos_robot (x,y,z) cm', 'yaw deg', 'maj x min cm', 'h cm');
    % for i = 1:numel(cubes)
    %     c = cubes(i);
    %     fprintf('  %-7s  (%5.1f,%5.1f,%5.1f)  %7.1f   %4.1f x %4.1f    %4.1f\n', ...
    %         c.color, c.pos_robot, rad2deg(c.yaw_robot), c.major_cm, c.minor_cm, c.height_cm);
    % end
end

function success = local_pick_one(arm, cube, opts, place_counter)
    % Per-cube: pre-grasp, descend, clamp to minor-axis width, place in tray.
    success = false;
    pos = cube.pos_robot;
    pos(3) = opts.cube_z;                          % override noisy depth z
    phi = -pi/2;                                  % top-down

    % --- Decide clamp width from the cube's short axis -------------------
    minor = max(cube.minor_cm, 0.5);              % avoid degenerate zero
    major = max(cube.major_cm, minor);
    grip_width = max(0, minor - opts.grip_overshoot);

    % --- Warn if the gripper's closure direction won't line up with the
    %     cube's minor axis. With this 4-DOF arm and a top-down approach,
    %     the gripper's lateral axis at (x,y) is fixed at perpendicular to
    %     the radial direction from the base. We cannot rotate it
    %     independently; only the cube's intrinsic orientation can help.
    radial_yaw   = atan2(pos(2), pos(1));
    gripper_yaw  = radial_yaw + pi/2;             % closure axis direction
    cube_minor_yaw = cube.yaw_robot + pi/2;       % minor-axis direction
    misalign_deg = abs(rad2deg(wrapToPi(gripper_yaw - cube_minor_yaw)));
    misalign_deg = min(misalign_deg, 180 - misalign_deg);   % mod 180

    if (major / minor) > 1.3 && misalign_deg > 25
        % Color-agnostic warn (no %s color label).
        fprintf(['  [warn] cube is non-square (%.1fx%.1f cm) and its ' ...
                 'minor axis is %.0f deg off from the gripper closure ' ...
                 'direction. Will close along the wrong axis -- bumping ' ...
                 'grip_width to fit the major axis.\n'], ...
                 major, minor, misalign_deg);
        % fprintf(['  [warn] cube %s is non-square (%.1fx%.1f cm) and its ' ...
        %          'minor axis is %.0f deg off from the gripper closure ' ...
        %          'direction. Will close along the wrong axis -- bumping ' ...
        %          'grip_width to fit the major axis.\n'], ...
        %          cube.color, major, minor, misalign_deg);
        grip_width = max(0, major - opts.grip_overshoot);
    end

    if grip_width > arm.gripperOpenWidth
        fprintf('  [skip] cube too wide for gripper (%.1f cm > %.1f cm)\n', ...
            grip_width, arm.gripperOpenWidth);
        % fprintf('  [skip] cube %s too wide for gripper (%.1f cm > %.1f cm)\n', ...
        %     cube.color, grip_width, arm.gripperOpenWidth);
        return;
    end

    % --- Reset to zero config (smooths out jerky first movement) ---------
    fprintf('  [reset] moving to zero joint config\n');
    arm.moveByJoints([0, -pi/2, 0, 0, 0]);
    pause(opts.pause_dur);

    % --- Pick (strict top-down; no auto-pitch fallback) -------------------
    fprintf('Picking cube at (%.1f, %.1f, %.1f) grip_width=%.1f cm\n', ...
        pos(1), pos(2), pos(3), grip_width);
    % fprintf('Picking %s at (%.1f, %.1f, %.1f) grip_width=%.1f cm\n', ...
    %     cube.color, pos(1), pos(2), pos(3), grip_width);

    arm.setCubePos(pos(1), pos(2), pos(3));
    pick_ok = arm.pickAndPlace(pos(1), pos(2), pos(3), phi, ...
                               opts.approach_dist, grip_width, opts.pause_dur);

    if ~pick_ok
        fprintf('  [fail] cube unreachable at phi=-pi/2.\n');
        return;
    end

    % --- Place gently in tray --------------------------------------------
    % Gripper holds the cube at its top (EE was at z = cube_z when grasping),
    % so the cube hangs ~cube_z below the EE. To set the cube DOWN on the
    % tray surface rather than drop it, the EE must descend to:
    %     EE_z = tray_surface_z + cube_height
    % which puts cube_bottom right at the tray surface. cube_height is
    % approximated by opts.cube_z (the standard cube top height above the
    % table). Stacking offset is added per cube in batch mode.
    cube_height = opts.cube_z;
    stack_dz   = cube_height * (place_counter - 1);
    place_z    = opts.tray_pos(3) + cube_height + stack_dz;

    fprintf('Placing in tray at (%.1f, %.1f, %.1f)\n', ...
        opts.tray_pos(1), opts.tray_pos(2), place_z);
    place_ok = arm.place(opts.tray_pos(1), opts.tray_pos(2), place_z, ...
                         phi, opts.approach_dist, opts.pause_dur);
    if ~place_ok
        fprintf('  [fail] place unreachable at phi=-pi/2.\n');
        return;
    end
    success = true;
end

function local_go_home(arm)
    arm.moveByJoints([0, 0, 0, 0, 0]);
end
