
classdef Arm < handle
    properties (SetAccess = private, GetAccess = public)
        x_current
        y_current
        z_current
        phi_current
        jointAngles   % 1x5 double
        grippingStatus
        isSimulated
    end

    properties (Access = public)
        show_four_IK            = false
        show_trajectory_interpolated = false
        show_debug_output       = false

        gripperOpenWidth = 6    % cm, finger spread when fully open

        a; d; alpha; theta; dhparams; lengths;
        arb;
    end

    properties (Access = private)
        robotStructure

        com_port
        speed = 100
        gripCloseness = 0       % 0 = open, 1 = fully closed

        cubePos = []            % 1x3 world position; empty = no cube in scene
        cubeHeld = false

        simFigure
        simAxes3D
        simAxesTop
        simAxesIK               % IK joint-space scatter (bottom-left)
        simAxesSide             % XZ side-view projection (bottom-right)
        simIKAnnotation         % retained for compatibility (unused)
        poseHistory

        lastIKSolutions         % raw candidates from most recent findJointAngles call
        lastValidSolutions      % subset that passed all constraint checks
    end

    methods (Access = public)
        function obj = Arm(mode, varargin)
            obj.isSimulated = strcmp(mode, 'sim');
            obj.jointAngles = zeros(1, 5);
            obj.grippingStatus = false;
            obj.poseHistory = [];

            obj.a      = [0,    10.5, 10.5, 11];
            obj.alpha  = [pi/2, 0,    0,    0 ];
            obj.d      = [13.7, 0,    0,    0 ];
            obj.theta  = [0,    0,    0,    0 ];

            obj.dhparams = [obj.a', obj.alpha', obj.d', obj.theta'];
            obj.lengths  = [obj.d(1), obj.a(2), obj.a(3), obj.a(4)];

            obj.robotStructure = obj.buildRobotStructure();

            if ~obj.isSimulated
                if isempty(varargin)
                    error('Arm: real mode requires a COM port, e.g. Arm(''real'', ''COM3'')');
                end
                obj.com_port = varargin{1};
                obj.speed    = 100;
                obj.arb = Arbotix('port', obj.com_port, 'nservos', 5);
            end
        end

        moveByJoints(obj, jointAnglesCommanded)
        success = moveByCoordinates(obj, x, y, z, phi)
        grip(obj)
        ungrip(obj)
        home(obj)
        success = pickAndPlace(obj, x, y, z, phi, approach_dist)
        success = place(obj, x, y, z, phi, approach_dist)

        function setCubePos(obj, x, y, z)
            obj.cubePos  = [x, y, z];
            obj.cubeHeld = false;
            if ~isempty(obj.simFigure) && isvalid(obj.simFigure)
                obj.updatePlot();
            end
        end

        function savePlot(obj, filename)
            if isempty(obj.poseHistory)
                warning('No pose history to plot.');
                return
            end
            f   = figure('Name', 'Arm Path Review', 'NumberTitle', 'off');
            ax3 = subplot(1,2,1, 'Parent', f);
            plot3(ax3, obj.poseHistory(:,1), obj.poseHistory(:,2), obj.poseHistory(:,3), ...
                'o-', 'LineWidth', 2);
            grid on; axis equal; view(3);
            title('End Effector Path — 3D'); xlabel('X'); ylabel('Y'); zlabel('Z');
            ax2 = subplot(1,2,2, 'Parent', f);
            plot(ax2, obj.poseHistory(:,1), obj.poseHistory(:,2), 'o-', 'LineWidth', 2);
            grid on; axis equal;
            title('End Effector Path — Top Down'); xlabel('X'); ylabel('Y');
            if nargin == 2
                saveas(f, filename);
                fprintf('Saved to %s\n', filename);
            end
        end

        function clearHistory(obj)
            obj.poseHistory = [];
        end
    end

    methods (Access = private)
        [T, Ts]       = DH(obj, config)
        collision     = checkSelfCollision(obj, solutions)
        withinLimits  = checkJointLimits(obj, solution)
        reachable     = isReachable(obj, x, y, z, phi)
        solutions     = findJointAngles(obj, x, y, z, phi)
        validSols     = findValidSolution(obj, solutions)
        best          = findSolution(obj, validSolutions)
        animateInterpolation(obj, startAngles, targetAngles)

        % ----------------------------------------------------------------
        function robot = buildRobotStructure(obj)
            numJoints = size(obj.dhparams, 1);
            robot = rigidBodyTree;
            robot.DataFormat = 'row';

            bodies = cell(numJoints, 1);
            joints = cell(numJoints, 1);

            for i = 1:numJoints
                bodies{i} = rigidBody(['body' num2str(i)]);
                joints{i} = rigidBodyJoint(['jnt' num2str(i)], 'revolute');
                setFixedTransform(joints{i}, obj.dhparams(i,:), 'dh');
                bodies{i}.Joint = joints{i};

                if i == 1
                    tform = axang2tform([1 0 0 pi/2]) * trvec2tform([0, 0, obj.lengths(i)/2]);
                    addCollision(bodies{i}, 'cylinder', [0.5, obj.lengths(i)], tform);
                    addBody(robot, bodies{i}, 'base');
                else
                    tform = trvec2tform([-obj.lengths(i)/2, 0, 0]) * axang2tform([0 1 0 pi/2]);
                    addCollision(bodies{i}, 'cylinder', [0.5, obj.lengths(i)], tform);
                    addBody(robot, bodies{i}, bodies{i-1}.Name);
                end
            end

            % Gripper palm — fixed at end of last arm link
            palmBody = rigidBody('gripper_palm');
            palmJoint = rigidBodyJoint('jnt_palm', 'fixed');
            setFixedTransform(palmJoint, eye(4));
            palmBody.Joint = palmJoint;
            addCollision(palmBody, 'box', [0.5, obj.gripperOpenWidth + 2, 0.5], eye(4));
            addBody(robot, palmBody, bodies{numJoints}.Name);

            % Finger 1 (+Y side, nominal open position)
            f1Body = rigidBody('gripper_finger1');
            f1Joint = rigidBodyJoint('jnt_gf1', 'fixed');
            setFixedTransform(f1Joint, trvec2tform([2, obj.gripperOpenWidth/2, 0]));
            f1Body.Joint = f1Joint;
            addCollision(f1Body, 'box', [4, 0.8, 0.8], eye(4));
            addBody(robot, f1Body, palmBody.Name);

            % Finger 2 (-Y side, nominal open position)
            f2Body = rigidBody('gripper_finger2');
            f2Joint = rigidBodyJoint('jnt_gf2', 'fixed');
            setFixedTransform(f2Joint, trvec2tform([2, -obj.gripperOpenWidth/2, 0]));
            f2Body.Joint = f2Joint;
            addCollision(f2Body, 'box', [4, 0.8, 0.8], eye(4));
            addBody(robot, f2Body, palmBody.Name);
        end

        % ----------------------------------------------------------------
        function ensurePlot(obj)
            if isempty(obj.simFigure) || ~isvalid(obj.simFigure)
                obj.initPlot();
            end
        end

        % ----------------------------------------------------------------
        function initPlot(obj)
            R = (13.7 + 10.5 + 10.5 + 11) * 1.5;   % fixed half-span, cm

            obj.simFigure = figure('Name', 'Arm Visualizer', 'NumberTitle', 'off', ...
                'Position', [100 50 1200 900]);

            % Top-left: 3D view
            obj.simAxes3D = axes('Parent', obj.simFigure, ...
                'Position', [0.05, 0.53, 0.44, 0.44]);
            hold(obj.simAxes3D, 'on'); grid(obj.simAxes3D, 'on');
            view(obj.simAxes3D, 3);
            title(obj.simAxes3D, '3D View');
            xlabel(obj.simAxes3D, 'X (cm)');
            ylabel(obj.simAxes3D, 'Y (cm)');
            zlabel(obj.simAxes3D, 'Z (cm)');
            xlim(obj.simAxes3D, [-R R]); ylim(obj.simAxes3D, [-R R]); zlim(obj.simAxes3D, [0 R]);
            axis(obj.simAxes3D, 'manual');

            % Top-right: Top-down view (XY)
            obj.simAxesTop = axes('Parent', obj.simFigure, ...
                'Position', [0.55, 0.53, 0.42, 0.44]);
            hold(obj.simAxesTop, 'on'); grid(obj.simAxesTop, 'on');
            view(obj.simAxesTop, 2);
            title(obj.simAxesTop, 'Top Down  (+X right, +Y up)');
            xlabel(obj.simAxesTop, 'X (cm)');
            ylabel(obj.simAxesTop, 'Y (cm)');
            xlim(obj.simAxesTop, [-R R]); ylim(obj.simAxesTop, [-R R]);
            axis(obj.simAxesTop, 'manual');

            % Bottom-left: IK joint-space (theta1 vs theta2)
            obj.simAxesIK = axes('Parent', obj.simFigure, ...
                'Position', [0.05, 0.06, 0.44, 0.42]);
            hold(obj.simAxesIK, 'on'); grid(obj.simAxesIK, 'on');
            title(obj.simAxesIK, 'IK Solutions  (\theta_1 vs \theta_2)');
            xlabel(obj.simAxesIK, '\theta_1 (deg)');
            ylabel(obj.simAxesIK, '\theta_2 (deg)');
            xlim(obj.simAxesIK, [-180 180]); ylim(obj.simAxesIK, [-180 180]);
            axis(obj.simAxesIK, 'manual');

            % Bottom-right: Side view (XZ)
            obj.simAxesSide = axes('Parent', obj.simFigure, ...
                'Position', [0.55, 0.06, 0.42, 0.42]);
            hold(obj.simAxesSide, 'on'); grid(obj.simAxesSide, 'on');
            title(obj.simAxesSide, 'Side View (XZ)');
            xlabel(obj.simAxesSide, 'X (cm)');
            ylabel(obj.simAxesSide, 'Z (cm)');
            xlim(obj.simAxesSide, [-R R]); ylim(obj.simAxesSide, [0 R]);
            axis(obj.simAxesSide, 'manual');

            obj.simIKAnnotation = [];

            obj.updatePlot();
        end

        % ----------------------------------------------------------------
        function updatePlot(obj)
            if isempty(obj.simFigure) || ~isvalid(obj.simFigure)
                return
            end

            R        = (13.7 + 10.5 + 10.5 + 11) * 1.5;
            armColor = [0.2 0.4 0.8];
            gc       = [0.0 0.30 1.0];   % gripper blue

            [T_ee, Ts] = obj.DH();

            % Collect joint-frame origins
            n = length(Ts);
            positions = zeros(n + 1, 3);
            for i = 1:n
                positions(i+1,:) = Ts{i}(1:3, 4)';
            end
            endEffector = positions(end,:);
            obj.poseHistory = [obj.poseHistory; endEffector];

            % Gripper geometry — finger tips land AT the EE (IK target = finger tips)
            lateral  = T_ee(1:3, 2);
            approach = T_ee(1:3, 1);
            ee_p     = T_ee(1:3, 4);
            aperture = obj.gripperOpenWidth * (1 - obj.gripCloseness);
            flen     = 3;                          % finger length, cm
            palm_p   = ee_p - flen * approach;     % palm bar, flen behind EE
            f1b = palm_p + (aperture/2) * lateral;
            f2b = palm_p - (aperture/2) * lateral;
            f1t = f1b + flen * approach;           % = ee_p + (aperture/2)*lateral
            f2t = f2b + flen * approach;           % = ee_p - (aperture/2)*lateral

            cla(obj.simAxes3D);
            cla(obj.simAxesTop);
            cla(obj.simAxesSide);

            % ---- Cube ------------------------------------------------
            if ~isempty(obj.cubePos) || obj.cubeHeld
                if obj.cubeHeld
                    cc = T_ee(1:3, 4)';
                else
                    cc = obj.cubePos;
                end
                hs = 2;
                verts = cc + hs * [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; ...
                                   -1 -1  1; 1 -1  1; 1  1  1; -1  1  1];
                faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
                patch(obj.simAxes3D, 'Vertices', verts, 'Faces', faces, ...
                    'FaceColor', [0.95 0.60 0.15], 'FaceAlpha', 0.85, ...
                    'EdgeColor', [0.4 0.2 0.0], 'LineWidth', 0.8);
                fill(obj.simAxesTop, ...
                    cc(1) + hs*[-1  1  1 -1], cc(2) + hs*[-1 -1  1  1], ...
                    [0.95 0.60 0.15], 'FaceAlpha', 0.85, 'EdgeColor', [0.4 0.2 0.0], 'LineWidth', 0.8);
                fill(obj.simAxesSide, ...
                    cc(1) + hs*[-1  1  1 -1], cc(3) + hs*[-1 -1  1  1], ...
                    [0.95 0.60 0.15], 'FaceAlpha', 0.85, 'EdgeColor', [0.4 0.2 0.0], 'LineWidth', 0.8);
            end

            % ---- Arm + gripper (3D) ----------------------------------
            plot3(obj.simAxes3D, positions(:,1), positions(:,2), positions(:,3), ...
                'o-', 'LineWidth', 2, 'MarkerSize', 6, 'Color', armColor, 'MarkerFaceColor', armColor);
            plot3(obj.simAxes3D, endEffector(1), endEffector(2), endEffector(3), 'r*', 'MarkerSize', 10);
            if size(obj.poseHistory, 1) > 1
                plot3(obj.simAxes3D, obj.poseHistory(:,1), obj.poseHistory(:,2), obj.poseHistory(:,3), ...
                    '--', 'Color', [0.6 0.6 0.6]);
            end
            plot3(obj.simAxes3D, [f1b(1) f2b(1)], [f1b(2) f2b(2)], [f1b(3) f2b(3)], '-', 'Color', gc, 'LineWidth', 2.5);
            plot3(obj.simAxes3D, [f1b(1) f1t(1)], [f1b(2) f1t(2)], [f1b(3) f1t(3)], '-', 'Color', gc, 'LineWidth', 2.5);
            plot3(obj.simAxes3D, [f2b(1) f2t(1)], [f2b(2) f2t(2)], [f2b(3) f2t(3)], '-', 'Color', gc, 'LineWidth', 2.5);

            % ---- Arm + gripper (top-down / XY) ----------------------
            plot(obj.simAxesTop, positions(:,1), positions(:,2), ...
                'o-', 'LineWidth', 2, 'MarkerSize', 6, 'Color', armColor, 'MarkerFaceColor', armColor);
            plot(obj.simAxesTop, endEffector(1), endEffector(2), 'r*', 'MarkerSize', 10);
            if size(obj.poseHistory, 1) > 1
                plot(obj.simAxesTop, obj.poseHistory(:,1), obj.poseHistory(:,2), '--', 'Color', [0.6 0.6 0.6]);
            end
            plot(obj.simAxesTop, [f1b(1) f2b(1)], [f1b(2) f2b(2)], '-', 'Color', gc, 'LineWidth', 2.5);
            plot(obj.simAxesTop, [f1b(1) f1t(1)], [f1b(2) f1t(2)], '-', 'Color', gc, 'LineWidth', 2.5);
            plot(obj.simAxesTop, [f2b(1) f2t(1)], [f2b(2) f2t(2)], '-', 'Color', gc, 'LineWidth', 2.5);

            % ---- Arm + gripper (side / XZ) --------------------------
            plot(obj.simAxesSide, [-R R], [0 0], '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 1);
            plot(obj.simAxesSide, positions(:,1), positions(:,3), ...
                'o-', 'LineWidth', 2, 'MarkerSize', 6, 'Color', armColor, 'MarkerFaceColor', armColor);
            plot(obj.simAxesSide, endEffector(1), endEffector(3), 'r*', 'MarkerSize', 10);
            if size(obj.poseHistory, 1) > 1
                plot(obj.simAxesSide, obj.poseHistory(:,1), obj.poseHistory(:,3), '--', 'Color', [0.6 0.6 0.6]);
            end
            plot(obj.simAxesSide, [f1b(1) f2b(1)], [f1b(3) f2b(3)], '-', 'Color', gc, 'LineWidth', 2.5);
            plot(obj.simAxesSide, [f1b(1) f1t(1)], [f1b(3) f1t(3)], '-', 'Color', gc, 'LineWidth', 2.5);
            plot(obj.simAxesSide, [f2b(1) f2t(1)], [f2b(3) f2t(3)], '-', 'Color', gc, 'LineWidth', 2.5);

            % ---- Restore fixed limits after cla ---------------------
            xlim(obj.simAxes3D,   [-R R]); ylim(obj.simAxes3D,   [-R R]); zlim(obj.simAxes3D,   [0 R]);
            xlim(obj.simAxesTop,  [-R R]); ylim(obj.simAxesTop,  [-R R]);
            xlim(obj.simAxesSide, [-R R]); ylim(obj.simAxesSide, [0 R]);
            axis(obj.simAxes3D,   'manual');
            axis(obj.simAxesTop,  'manual');
            axis(obj.simAxesSide, 'manual');

            % ---- IK joint-space plot --------------------------------
            obj.updateIKAxes();

            drawnow;
        end

        % ----------------------------------------------------------------
        function updateIKAxes(obj)
            if isempty(obj.simAxesIK) || ~isvalid(obj.simAxesIK)
                return
            end
            ax = obj.simAxesIK;
            cla(ax);

            % Joint-limit shading — |theta| > 150 deg is invalid
            lim = 150;
            shadeclr = [0.93 0.78 0.78];
            patch(ax, [-180  180  180 -180], [ lim  lim 180 180], shadeclr, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
            patch(ax, [-180  180  180 -180], [-180 -180 -lim -lim], shadeclr, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
            patch(ax, [ lim  180  180  lim], [-180 -180 180 180], shadeclr, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
            patch(ax, [-180 -lim -lim -180], [-180 -180 180 180], shadeclr, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
            xline(ax,    0, '--', 'Color', [0.75 0.75 0.75]);
            yline(ax,    0, '--', 'Color', [0.75 0.75 0.75]);
            xline(ax,  lim, ':',  'Color', [0.80 0.25 0.25]);
            xline(ax, -lim, ':',  'Color', [0.80 0.25 0.25]);
            yline(ax,  lim, ':',  'Color', [0.80 0.25 0.25]);
            yline(ax, -lim, ':',  'Color', [0.80 0.25 0.25]);

            if isempty(obj.lastIKSolutions)
                text(ax, 0, 0, 'No IK call yet', 'HorizontalAlignment', 'center', 'Color', [0.5 0.5 0.5]);
            else
                selected = obj.jointAngles(1:4);
                for s = 1:size(obj.lastIKSolutions, 1)
                    row = obj.lastIKSolutions(s,:);
                    t1d = rad2deg(row(1));
                    t2d = rad2deg(row(2));

                    isValid = ~isempty(obj.lastValidSolutions) && ...
                        any(all(abs(obj.lastValidSolutions - row) < 1e-6, 2));
                    isSel   = all(abs(selected - row) < 1e-6);

                    if isSel
                        clr = [0.05 0.72 0.05]; mk = 'p'; msz = 18; lw = 2.0;
                    elseif isValid
                        clr = [0.10 0.50 1.00]; mk = 'o'; msz = 11; lw = 1.5;
                    else
                        clr = [0.60 0.60 0.60]; mk = 'x'; msz = 11; lw = 1.5;
                    end

                    plot(ax, t1d, t2d, mk, 'Color', clr, 'MarkerFaceColor', clr, ...
                        'MarkerSize', msz, 'LineWidth', lw);
                    text(ax, t1d + 4, t2d + 4, sprintf('S%d', s), 'FontSize', 7, 'Color', clr);
                end
            end

            xlim(ax, [-180 180]); ylim(ax, [-180 180]);
            axis(ax, 'manual');
        end

    end
end
