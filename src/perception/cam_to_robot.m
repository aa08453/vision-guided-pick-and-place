function p_robot = cam_to_robot(p_cam_m, T)
    % CAM_TO_ROBOT  Convert camera-frame points (meters) to robot-frame (cm).
    %
    % p_cam_m : N x 3 points in the depth-camera frame, in meters
    % T       : 4x4 rigid transform from CM-camera-frame to CM-robot-frame
    %           (produced by calibrate_camera.m)
    % p_robot : N x 3 points in the robot base frame, in centimeters
    %
    % The unit conversion (m -> cm) is applied INSIDE this function so the
    % stored transform T is a pure rigid (rotation + translation) in cm-space
    % -- which is what Kabsch returns when fed cm coordinates on both sides.

    if isempty(p_cam_m)
        p_robot = zeros(0, 3);
        return;
    end


    % Promote to double up front. pcfromdepth gives `single` Location data,
    % which propagates through the arm's IK and breaks checkCollision (which
    % strictly requires double).
    p_cam_cm = double(p_cam_m) * 100;
    N = size(p_cam_cm, 1);
    p_h = [p_cam_cm, ones(N, 1)];      % N x 4
    


    p_robot_h = (inv(T) * p_h')';            % N x 4
    p_robot = p_robot_h(:, 1:3);
end
