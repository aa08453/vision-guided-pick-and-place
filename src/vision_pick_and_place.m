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
    p.addParameter('calibration_file', 'camera_transform.mat',@(s)ischar(s)||isstring(s));
    p.addParameter('tray_pos',         [20, -15, 5],          @(v)isnumeric(v)&&numel(v)==3);
    p.addParameter('approach_dist',    7,                     @isnumeric);
    p.addParameter('grip_overshoot',   0.7,                   @isnumeric);
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
        T_cam_to_rob = S.T;
        fprintf('Loaded camera->robot transform from %s\n', opts.calibration_file);
    else
        warning('vision_pick_and_place:noCal', ...
            ['Calibration file %s not found. Using identity transform -- ' ...
             'picks WILL go to wrong coordinates until you run calibrate_camera.'], ...
            opts.calibration_file);
        T_cam_to_rob = eye(4);
    end

    % ---- Camera pipeline lifecycle -----------------------------------------
    pipe = [];
    cleanup = onCleanup(@() local_cleanup(pipe));

    if strcmp(opts.source, 'realsense')
        pipe = realsense.pipeline();
        pipe.start();
        fprintf('RealSense pipeline started. Warming up...\n');
        for i = 1:10, pipe.wait_for_frames(); end
    end

    % ---- Interactive loop ---------------------------------------------------
    place_counter = 0;
    while true
        % --- Snapshot
        try
            ptCloud = local_capture(opts, pipe);
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
function ptCloud = local_capture(opts, pipe)
    if strcmp(opts.source, 'file')
        if isempty(opts.filepath) || ~isfile(opts.filepath)
            error('source=''file'' requires a valid ''filepath''.');
        end
        ptCloud = pcread(opts.filepath);
    else
        % RealSense organized cloud via pcfromdepth + intrinsics
        align_to_color = realsense.align(realsense.stream.color);
        fs = pipe.wait_for_frames();
        fs = align_to_color.process(fs);

        depth = fs.get_depth_frame();
        color = fs.get_color_frame();

        depth_data = double(depth.get_data());
        depth_frame = permute(reshape(depth_data', [depth.get_width(), depth.get_height()]), [2 1]);
        color_data = color.get_data();
        I = permute(reshape(color_data', [3, color.get_width(), color.get_height()]), [3 2 1]);

        profile = pipe.get_active_profile();
        depth_sensor = profile.get_device().first('depth_sensor');
        depth_scaling = depth_sensor.get_depth_scale();
        depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
        depth_intrinsics = depth_stream.get_intrinsics();

        intrinsics = cameraIntrinsics([depth_intrinsics.fx, depth_intrinsics.fy], ...
                                      [depth_intrinsics.ppx, depth_intrinsics.ppy], ...
                                      size(depth_frame));
        ptCloud = pcfromdepth(depth_frame, 1/depth_scaling, intrinsics, ...
                              'ColorImage', I);
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
    fprintf('  %-7s  %-21s  %-8s  %-12s  %-5s\n', ...
        'color', 'pos_robot (x,y,z) cm', 'yaw deg', 'maj x min cm', 'h cm');
    for i = 1:numel(cubes)
        c = cubes(i);
        fprintf('  %-7s  (%5.1f,%5.1f,%5.1f)  %7.1f   %4.1f x %4.1f    %4.1f\n', ...
            c.color, c.pos_robot, rad2deg(c.yaw_robot), c.major_cm, c.minor_cm, c.height_cm);
    end
end

function success = local_pick_one(arm, cube, opts, place_counter)
    % Per-cube: pre-grasp, descend, clamp to minor-axis width, place in tray.
    success = false;
    pos = cube.pos_robot;
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
        fprintf(['  [warn] cube %s is non-square (%.1fx%.1f cm) and its ' ...
                 'minor axis is %.0f deg off from the gripper closure ' ...
                 'direction. Will close along the wrong axis -- bumping ' ...
                 'grip_width to fit the major axis.\n'], ...
                 cube.color, major, minor, misalign_deg);
        grip_width = max(0, major - opts.grip_overshoot);
    end

    if grip_width > arm.gripperOpenWidth
        fprintf('  [skip] cube %s too wide for gripper (%.1f cm > %.1f cm)\n', ...
            cube.color, grip_width, arm.gripperOpenWidth);
        return;
    end

    % --- Pick
    fprintf('Picking %s at (%.1f, %.1f, %.1f) grip_width=%.1f cm\n', ...
        cube.color, pos(1), pos(2), pos(3), grip_width);

    arm.setCubePos(pos(1), pos(2), pos(3));
    pick_ok = arm.pickAndPlace(pos(1), pos(2), pos(3), phi, ...
                               opts.approach_dist, grip_width);
    if ~pick_ok
        fprintf('  [fail] pick unreachable.\n');
        return;
    end

    % --- Place in tray (small vertical stack so batch picks don't collide)
    stack_dz = 2 * (place_counter - 1);           % 2 cm per layer
    tray_z = opts.tray_pos(3) + stack_dz;
    fprintf('Placing in tray at (%.1f, %.1f, %.1f)\n', ...
        opts.tray_pos(1), opts.tray_pos(2), tray_z);
    place_ok = arm.place(opts.tray_pos(1), opts.tray_pos(2), tray_z, ...
                         phi, opts.approach_dist);
    if ~place_ok
        fprintf('  [fail] place unreachable.\n');
        return;
    end
    success = true;
end

function local_go_home(arm)
    arm.moveByCoordinates(0, 0, 25, NaN);
end
