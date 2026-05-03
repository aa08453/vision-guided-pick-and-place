% Drives the arm to four points on the circumference of a circle
clc; clear; close all;

r   = 20;   % radius (cm) — tune to your workspace
z   = 12;   % fixed height (cm)
phi = NaN;   % NaN = auto-compute pitch, or set e.g. -pi/2 to fix it

% Four points at 45/135/225/315 degrees — cardinal diagonals of the circle
angles_deg = [45, 135, 225, 315];
targets = [r*cosd(angles_deg); r*sind(angles_deg)]';  % 4x2

arm = Arm('sim');   % swap to Arm('real', 'COM3') for hardware

for i = 1:size(targets, 1)
    x = targets(i, 1);
    y = targets(i, 2);

    fprintf('Moving to point %d: (%.1f, %.1f, %.1f)\n', i, x, y, z);

    success = arm.moveByCoordinates(x, y, z, phi);
    disp(arm.jointAngles)

    if ~success
        fprintf('  WARNING: point %d unreachable — skipping\n', i);
        continue
    end

    pause(1.0);
end

% arm.savePlot('circle_run.png');
% fprintf('Done. Plot saved.\n');
