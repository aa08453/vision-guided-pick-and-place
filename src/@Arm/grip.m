function grip(obj, target_width_cm)
    % GRIP  Close the gripper.
    %   grip()                -- fully close (default, backwards-compatible)
    %   grip(target_width_cm) -- close to a target finger spread (cm)
    %
    % For variable-width gripping, pass the cube's narrow-axis dimension
    % MINUS a small overshoot so the servo actually applies clamping force
    % (e.g., 0.5-1.0 cm under the measured cube width).
    %
    % Servo mapping (Arbotix joint 5):
    %   setpos(5, 0)   == fully open  (~ gripperOpenWidth)
    %   setpos(5, 1.5) == fully closed (~ 0 cm)
    % Linear interpolation between those endpoints.

    if nargin < 2 || isempty(target_width_cm) || ~isfinite(target_width_cm)
        target_width_cm = 0;       % default: fully close
    end
    target_width_cm = max(0, min(target_width_cm, obj.gripperOpenWidth));

    closeness = 1 - target_width_cm / obj.gripperOpenWidth;   % 0..1
    obj.gripCloseness  = closeness;
    obj.grippingStatus = true;

    % Attach the cube if the end-effector is within reach
    if ~isempty(obj.cubePos) && ~obj.cubeHeld
        [T, ~] = obj.DH();
        ee_pos = T(1:3, 4)';
        if norm(ee_pos - obj.cubePos) < obj.gripperOpenWidth + 2
            obj.cubeHeld = true;
        end
    end

    if ~obj.isSimulated
        servo_pos = 1.5 * closeness;
        obj.arb.setpos(5, servo_pos);
    end

    obj.ensurePlot();
    obj.updatePlot();
end
