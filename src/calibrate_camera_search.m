function T = calibrate_camera_search(known, varargin)
    % CALIBRATE_CAMERA_SEARCH  Brute-force calibration over the 24
    % axis-aligned rotations and all cube->position assignments. Picks the
    % (rotation, assignment) pair whose predicted cube positions land
    % closest to the known robot positions.
    %
    % Why this works when Kabsch failed: with all 4 cubes co-planar at the
    % same z, full SVD Kabsch is rank-deficient and gives an ambiguous Z
    % column. Restricting R to the 24 "axis-permutation with signs"
    % rotations (the proper rotation symmetries of a cube) makes the search
    % discrete and well-conditioned. For each candidate (R, assignment), the
    % optimal translation is closed-form: t = mean(robot) - R * mean(cam).
    %
    % Args:
    %   known       N-element struct array (N >= 3) with field:
    %                 .robot   1x3 cm, cube position in robot frame
    %               Color does not matter; correspondence between detected
    %               cubes and known positions is searched over all N!
    %               orderings (or nchoosek(M,N)*N! if more cubes are seen
    %               than positions are given).
    %
    % Optional name-value pairs:
    %   'ptCloud'    pre-captured organized point cloud (skips RealSense)
    %   'save_path'  default 'calib.mat'; '' to skip save
    %   'tolerance'  acceptable RMS residual in cm (default 2.0). Warns
    %                if even the best candidate exceeds this.
    %
    % Returns:
    %   T  4x4 transform such that p_robot_cm = T * [p_cam_cm; 1].
    %      Drop-in compatible with cam_to_robot.m / vision_pick_and_place.

    p = inputParser;
    p.addParameter('ptCloud',   [],             @(v) isempty(v) || isa(v,'pointCloud'));
    p.addParameter('save_path', 'calib.mat',    @(s) ischar(s) || isstring(s));
    p.addParameter('tolerance', 2.0,            @isnumeric);
    p.parse(varargin{:});
    opts = p.Results;

    % ---- 1. Acquire point cloud --------------------------------------------
    if isempty(opts.ptCloud)
        fprintf('Opening RealSense at 640x480...\n');
        ptCloud = local_capture_ptcloud();
    else
        ptCloud = opts.ptCloud;
    end

    % ---- 2. Detect cubes (pos_cam comes back in meters) --------------------
    cubes = snapshot_cubes(ptCloud, eye(4));
    fprintf('Detected %d cube(s):\n', numel(cubes));
    for i = 1:numel(cubes)
        fprintf('  #%d  cam=(%6.3f, %6.3f, %6.3f) m\n', ...
            i, cubes(i).pos_cam);
    end

    % ---- 3. Gather positions in cm -----------------------------------------
    N = numel(known);
    if N < 3
        error('calibrate_camera_search: need >= 3 known cubes, got %d', N);
    end
    M = numel(cubes);
    if M < N
        error(['Detected only %d cube(s) but %d known positions were given. ' ...
               'Place more cubes or remove entries from `known`.'], M, N);
    end

    P_rob_cm = zeros(N, 3);
    for i = 1:N
        P_rob_cm(i, :) = known(i).robot;
    end
    P_cam_all_cm = zeros(M, 3);
    for j = 1:M
        P_cam_all_cm(j, :) = cubes(j).pos_cam * 100;   % m -> cm
    end

    % ---- 4. Enumerate (assignment, rotation) candidates, score each --------
    Rs          = enumerate_24_rotations();
    combos      = nchoosek(1:M, N);    % which N detected cubes to use
    perms_N     = perms(1:N);          % how to order them onto known(1..N)
    c_rob       = mean(P_rob_cm, 1);

    results = struct('R', {}, 't', {}, 'rms', {}, 'max_err', {}, 'assign', {});

    for a = 1:size(combos, 1)
        chosen = combos(a, :);
        for pp = 1:size(perms_N, 1)
            order = chosen(perms_N(pp, :));
            P_cam_cm = P_cam_all_cm(order, :);
            c_cam    = mean(P_cam_cm, 1);

            for k = 1:numel(Rs)
                R    = Rs{k};
                t    = c_rob' - R * c_cam';
                pred = (R * P_cam_cm')' + t';
                err  = vecnorm(pred - P_rob_cm, 2, 2);

                results(end+1).R       = R;                %#ok<AGROW>
                results(end).t         = t;
                results(end).rms       = sqrt(mean(err.^2));
                results(end).max_err   = max(err);
                results(end).assign    = order;
            end
        end
    end

    [~, ord] = sort([results.rms]);
    results  = results(ord);

    % ---- 5. Report the top candidates --------------------------------------
    fprintf('\nTop candidates (by RMS residual):\n');
    fprintf('  rank   rms_cm   max_cm   detected_cube_order\n');
    for k = 1:min(8, numel(results))
        r = results(k);
        fprintf('  %4d  %7.3f  %7.3f   [%s]\n', ...
            k, r.rms, r.max_err, num2str(r.assign));
    end

    best = results(1);
    T = [best.R, best.t; 0 0 0 1];

    fprintf('\nBest T_cam_to_robot:\n');
    disp(T);

    % ---- 6. Per-cube reality check -----------------------------------------
    P_cam_best = P_cam_all_cm(best.assign, :);
    pred = (best.R * P_cam_best')' + best.t';
    fprintf('Per-cube residuals:\n');
    for i = 1:N
        e = norm(pred(i,:) - P_rob_cm(i,:));
        fprintf('  detected#%d -> known(%d)  pred=(%5.2f, %5.2f, %5.2f)  known=(%5.2f, %5.2f, %5.2f)  err=%.2f cm\n', ...
            best.assign(i), i, pred(i,:), P_rob_cm(i,:), e);
    end
    fprintf('  RMS = %.3f cm   max = %.3f cm\n', best.rms, best.max_err);

    if best.rms > opts.tolerance
        warning(['Best RMS %.2f cm exceeds tolerance %.2f cm. Likely causes:\n' ...
                 '  (1) `known.robot` positions don''t match physical placement,\n' ...
                 '  (2) detector mis-segmented one of the cubes,\n' ...
                 '  (3) camera has significant non-axis-aligned tilt (>5 deg) - ' ...
                 'use ArUco-based calibration for arbitrary mount angles.'], ...
                 best.rms, opts.tolerance);
    end

    % ---- 7. Save ------------------------------------------------------------
    if ~isempty(opts.save_path)
        save(opts.save_path, 'T');
        fprintf('\nSaved best T to %s\n', opts.save_path);
    end
end


% =========================================================================
% Helpers
% =========================================================================
function Rs = enumerate_24_rotations()
    % All 24 right-handed (det = +1) rotation matrices whose columns are
    % axis-aligned unit vectors with possible sign flips. This is the
    % rotation group of a cube.
    Rs = {};
    perm_list = perms([1 2 3]);     % 6 row orderings
    for p = 1:size(perm_list, 1)
        prm = perm_list(p, :);
        for s1 = [-1, 1]
        for s2 = [-1, 1]
        for s3 = [-1, 1]
            R = zeros(3);
            R(1, prm(1)) = s1;
            R(2, prm(2)) = s2;
            R(3, prm(3)) = s3;
            if abs(det(R) - 1) < 1e-6
                Rs{end+1} = R; %#ok<AGROW>
            end
        end
        end
        end
    end
end


function ptCloud = local_capture_ptcloud()
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

    depth = fs.get_depth_frame();
    color = fs.get_color_frame();

    depth_data  = double(depth.get_data());
    depth_frame = permute(reshape(depth_data', ...
        [depth.get_width(), depth.get_height()]), [2 1]);
    color_data  = color.get_data();
    I           = permute(reshape(color_data', ...
        [3, color.get_width(), color.get_height()]), [3 2 1]);

    depth_sensor   = profile.get_device().first('depth_sensor');
    depth_scaling  = depth_sensor.get_depth_scale();
    depth_stream   = profile.get_stream(realsense.stream.depth) ...
                            .as('video_stream_profile');
    di             = depth_stream.get_intrinsics();
    intrinsics     = cameraIntrinsics([di.fx, di.fy], [di.ppx, di.ppy], ...
                                       size(depth_frame));
    ptCloud = pcfromdepth(depth_frame, 1/depth_scaling, intrinsics, ...
                          'ColorImage', I);
end
