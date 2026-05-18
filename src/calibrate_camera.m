function T = calibrate_camera(ptCloud, known_positions, save_path)
    % CALIBRATE_CAMERA  Solve the rigid camera -> robot transform from 4 cubes.
    %
    %   T = calibrate_camera(ptCloud, known_positions)
    %   T = calibrate_camera(ptCloud, known_positions, save_path)
    %
    % Procedure:
    %   1. Place 4 differently-colored cubes at known (x,y,z) positions in the
    %      robot's frame (cm). Use one of each: yellow, red, green, blue.
    %   2. Capture one point-cloud snapshot from the RealSense.
    %   3. Call this function. It runs the segmenter, matches each cube to its
    %      known robot position by color, and solves a rigid transform via
    %      Kabsch (SVD).
    %
    % known_positions is a 4-element struct array with fields:
    %   .color (string)  -- must be one of "yellow","red","green","blue"
    %   .robot (1x3)     -- (x,y,z) in centimeters, robot base frame
    %
    % The returned (and optionally saved) T is a 4x4 transform that converts
    % CENTIMETERS in the camera frame to CENTIMETERS in the robot frame. The
    % wrapper cam_to_robot.m handles the m -> cm conversion at use time.
    %
    % If save_path is given, T is written there as variable 'T'.

    if numel(known_positions) ~= 4
        error('calibrate_camera: need exactly 4 known cubes, got %d', numel(known_positions));
    end

    % Run segmentation with an identity transform just to get camera-frame
    % centroids. (snapshot_cubes' pos_cam is independent of the transform.)
    T_identity = eye(4);
    cubes = snapshot_cubes(ptCloud, T_identity, struct('show_debug', true));

    fprintf('\nCalibration: detected %d cubes in scene\n', numel(cubes));
    for i = 1:numel(cubes)
        fprintf('  [%d] %-7s  cam=(%6.3f, %6.3f, %6.3f) m\n', ...
            i, cubes(i).color, cubes(i).pos_cam);
    end

    % Match each known position to a detected cube by color
    P_cam_cm = zeros(4, 3);
    P_rob_cm = zeros(4, 3);
    for i = 1:4
        target = string(known_positions(i).color);
        hits = cubes(arrayfun(@(c) c.color == target, cubes));
        if numel(hits) ~= 1
            error(['calibrate_camera: expected exactly 1 %s cube in the ' ...
                   'frame, found %d. Re-arrange the scene so each color ' ...
                   'appears exactly once.'], target, numel(hits));
        end
        P_cam_cm(i, :) = hits.pos_cam * 100;     % m -> cm
        P_rob_cm(i, :) = known_positions(i).robot;
    end

    % --- Kabsch (rigid alignment) -------------------------------------------
    c_cam = mean(P_cam_cm, 1);
    c_rob = mean(P_rob_cm, 1);
    H = (P_cam_cm - c_cam)' * (P_rob_cm - c_rob);
    [U, ~, V] = svd(H);
    d = sign(det(V * U'));
    R = V * diag([1, 1, d]) * U';
    t = c_rob' - R * c_cam';
    T = [R, t; 0 0 0 1];

    % --- Sanity check: residual error on the 4 calibration points -----------
    P_pred = (T * [P_cam_cm, ones(4,1)]')';
    err = vecnorm(P_pred(:, 1:3) - P_rob_cm, 2, 2);
    fprintf('\nCalibration residuals per point (cm):\n');
    for i = 1:4
        fprintf('  %-7s  err = %.3f cm\n', known_positions(i).color, err(i));
    end
    fprintf('  RMS residual: %.3f cm\n', sqrt(mean(err.^2)));
    if any(err > 2.0)
        warning(['Calibration residual exceeds 2 cm on at least one point. ' ...
                 'Check that the known_positions match the cube placement.']);
    end

    if nargin >= 3 && ~isempty(save_path)
        save(save_path, 'T');
        fprintf('\nSaved camera->robot transform to %s\n', save_path);
    end
end
