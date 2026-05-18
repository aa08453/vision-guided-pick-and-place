% RUN_CALIBRATE_SEARCH  Brute-force calibration runner.
%
% Place N cubes (N >= 3) at the positions listed below (in cm, robot base
% frame). Color does not matter -- correspondence is found by searching all
% assignments. Then run this script. It captures one RealSense frame,
% brute-forces the 24 axis-aligned rotation candidates and the N! cube
% orderings, picks the best fit, and saves the result to calib.mat in the
% current working directory.
%
% Edit the `known` block to match wherever you actually placed your cubes.

clc; close all;

% ---- 1. Tell the script where each cube physically sits ------------------
%      .robot = [x, y, z] in cm, robot base frame
%      z = cube top height (~ 2.5 cm for a 2.5 cm cube sitting on the table)
known = struct('robot', {});
known(1) = struct('robot', [-16,    0, 2.5]);
known(2) = struct('robot', [  0,   16, 2.5]);
known(3) = struct('robot', [  0, -16.5, 2.5]);

% ---- 2. Run the search ----------------------------------------------------
T = calibrate_camera_search(known, ...
        'save_path', 'C:\Users\itadmin\Desktop\vision-guided-pick-and-place\calib.mat', ...
        'tolerance', 2.0);

% ---- 3. Quick post-run sanity print --------------------------------------
fprintf('\nCalibration done. Camera origin in robot frame: (%.2f, %.2f, %.2f) cm\n', ...
    T(1:3, 4));
fprintf('Sanity-check that against where your camera lens physically sits.\n');
