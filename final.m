arm = Arm('real', 'COM5');

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

    %:\Users\itadmin\Desktop\vision-guided-pick-and-place\calib.mat'
close all;
vision_pick_and_place(arm, 'source', 'realsense','calibration_file', 'C' , 'tray_pos', [0, 16, 3], 'approach_dist', 5, 'grip_overshoot', 2, 'home_after_each', true, 'show_debug', true);
% vision_pick_and_place(arm, 'source', 'realsense', 'filepath', "NaN", 'tray_pos', [-17.5, 10, 1], 'approach_dist', 5, 'grip_overshoot', 1, 'home_after_each', true, 'show_debug', true);% vision_pick_and_place(arm, 'source', 'realsense', 'filepath', "NaN", 'calibration_file', 'C:\Users\itadmin\Desktop\vision-guided-pick-and-place\src\calib.mat', 'tray_pos', [-17.5, 10, 1], 'approach_dist', 5, 'grip_overshoot', 1, 'home_after_each', true, 'show_debug', true);
