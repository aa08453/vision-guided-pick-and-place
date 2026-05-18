function pose = compute_object_pose(pointCloud, instanceMap, object_id)
    % COMPUTE_OBJECT_POSE  Centroid + planar yaw for one instance.
    %
    % Mirrors the approach in
    % ../Intro-To-Robotics/perception_pipeline_3d_regionprops.m:
    %   - 3D centroid: mean of point-cloud points whose 2D pixel is in the mask
    %   - 2D orientation: regionprops Orientation of the binary mask
    %   - SE(3) yaw rotation about the camera's optical (Z) axis
    %
    % NOTE: pose is expressed in the CAMERA frame. Camera->world transform
    % (T_cam_to_world) is not applied here; see CLAUDE.md.
    %
    % Returns pose with fields .yaw .R .d .T, or an all-NaN sentinel
    % (pose.yaw = NaN) when the object has no valid 3D support.

    objMask = (instanceMap == object_id);

    % --- Extract 3D points for this instance ---------------------------------
    % Support both organized (H x W x 3) and unorganized (N x 3) point clouds.
    locs = pointCloud.Location;

    if ndims(locs) == 3
        [pcH, pcW, ~] = size(locs);
        if ~isequal(size(objMask), [pcH, pcW])
            fprintf(['Warning: instanceMap (%dx%d) does not match organized ' ...
                'point cloud (%dx%d). Skipping object %d.\n'], ...
                size(objMask,1), size(objMask,2), pcH, pcW, object_id);
            pose = empty_pose();
            return;
        end
        X = locs(:,:,1); Y = locs(:,:,2); Z = locs(:,:,3);
        idx = find(objMask);
        objPoints = [X(idx), Y(idx), Z(idx)];
    else
        if numel(objMask) ~= size(locs, 1)
            fprintf(['Warning: instanceMap has %d pixels but unorganized ' ...
                'point cloud has %d points. Skipping object %d.\n'], ...
                numel(objMask), size(locs,1), object_id);
            pose = empty_pose();
            return;
        end
        objPoints = locs(objMask(:), :);
    end

    valid = ~any(isnan(objPoints), 2) & ~any(isinf(objPoints), 2) ...
            & (objPoints(:,3) > 0);
    objPoints = objPoints(valid, :);

    if isempty(objPoints)
        fprintf('Warning: Object %d has no valid 3D points; skipping.\n', object_id);
        pose = empty_pose();
        return;
    end

    centroid = mean(objPoints, 1);

    % --- 2D orientation -> yaw about camera Z --------------------------------
    % regionprops Orientation is measured from the +x image axis, CCW positive
    % in image coords (where y points down). For a camera looking at the
    % workspace with Z forward and X/Y matching the image, negating gives the
    % CCW yaw about Z in the right-handed camera frame.
    rstats = regionprops(objMask, 'Orientation', 'Area');
    if ~isempty(rstats)
        [~, mi] = max([rstats.Area]);
        yaw = -deg2rad(rstats(mi).Orientation);
    else
        yaw = 0;
    end

    Rz = [cos(yaw) -sin(yaw) 0;
          sin(yaw)  cos(yaw) 0;
          0         0        1];
    T = [Rz, centroid'; 0 0 0 1];

    pose.yaw = yaw;
    pose.R   = Rz;
    pose.d   = centroid;
    pose.T   = T;

    fprintf('\n--- Object %d Pose (camera frame) ---\n', object_id);
    fprintf('Centroid (XYZ): [%.4f, %.4f, %.4f] m\n', centroid(1), centroid(2), centroid(3));
    fprintf('Yaw: %.2f deg\n', rad2deg(yaw));
end


function p = empty_pose()
    p.yaw = NaN;
    p.R   = eye(3);
    p.d   = [NaN NaN NaN];
    p.T   = [eye(3), [NaN; NaN; NaN]; 0 0 0 1];
end
