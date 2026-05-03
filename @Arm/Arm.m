
% This has the following attributes
% Real or simulated robot

% Robot DH parameters

% These are probably better stored as methods, if a real robot, then calculate it and update the private variable, and return if a sim robot then just read from before
% Current joint angles 
% Current FK coordinates
% Current commanded IK

% Public Variables:
%	- Whether to show 4 IK solutions (show_four_IK)
% 	- Whether to show the trajectory being interpolated (show_trajectory_interpolated)
% 	- Show debug output for the IK solutions (show_debug_output)
% 	- X, Y, Z
% 	- Current pitch (as phi)
%	- Joint angles
%	- Gripping Status

% Private Variables 
%	- 

classdef Arm < handle
    properties (SetAccess = private, GetAccess = public)
        % Computed/observed state — readable but not settable from outside
        x_current
        y_current
        z_current
        phi_current
        jointAngles   % 1x5 double
        grippingStatus
        isSimulated   % logical
    end

    properties (Access = public)
        % User-controlled display/debug flags
        show_four_IK            = false
        show_trajectory_interpolated = false
        show_debug_output       = false

        a; d; alpha; theta; dhparams; lengths;
    end

    properties (Access = private)
        % Robot definition
        robotStructure          % DH params, joint limits, link lengths

        % Hardware / communication
        com_port
        speed = 100
        grip_closeness

        % Sim visualisation handle (see below)
        simFigure               % handle to the current sim figure
        simAxes                 % handle to axes within it
        simAxes3D
        simAxesTop
        poseHistory % Saved plots

    end

    methods (Access = public)
        function obj = Arm(mode, varargin)
            % Usage:
            %   Arm('sim')
            %   Arm('real', 'COM3')
            %   Arm('real', 'COM3', speed)

            obj.isSimulated = strcmp(mode, 'sim');
            obj.jointAngles = zeros(1, 5);
            obj.grippingStatus = false;
            obj.poseHistory = [];


            obj.a      = [0,    10.5, 10.5, 11];
            obj.alpha  = [pi/2, 0,    0,    0 ];
            obj.d      = [13.7, 0,    0,    0 ];
            obj.theta  = [0,    0,    0,    0 ];

            % Build matrix using obj.* — not bare variable names
            obj.dhparams = [obj.a', obj.alpha', obj.d', obj.theta'];  % 4x4

            obj.lengths = [obj.d(1), obj.a(2), obj.a(3), obj.a(4)];




            % Build the rigidBodyTree — same for both modes
            obj.robotStructure = obj.buildRobotStructure();

            if ~obj.isSimulated
                if isempty(varargin)
                    error('Arm: real mode requires a COM port, e.g. Arm(''real'', ''COM3'')');
                end
                obj.com_port = varargin{1};
                obj.speed    = 100;  % default
                % Open serial connection here
                % obj.serialDevice = serialport(obj.com_port, 9600);
            end
        end



        moveByJoints(obj, jointAnglesCommanded) 
        success = moveByCoordinates(obj, x, y, z, phi)
        grip(obj)                                  
        ungrip(obj)                                
        home(obj)                                  

        function savePlot(obj, filename)
            % Creates a fresh figure from accumulated history — does not
            % touch the live sim figure. Call whenever you want a snapshot.
            if isempty(obj.poseHistory)
                warning('No pose history to plot.');
                return
            end

            f = figure('Name', 'Arm Path Review', 'NumberTitle', 'off');

            ax3 = subplot(1,2,1, 'Parent', f);
            plot3(ax3, obj.poseHistory(:,1), obj.poseHistory(:,2), obj.poseHistory(:,3), ...
                'o-', 'LineWidth', 2);
            grid on; axis equal; view(3);
            title('End Effector Path — 3D');
            xlabel('X'); ylabel('Y'); zlabel('Z');

            ax2 = subplot(1,2,2, 'Parent', f);
            plot(ax2, obj.poseHistory(:,1), obj.poseHistory(:,2), 'o-', 'LineWidth', 2);
            grid on; axis equal;
            title('End Effector Path — Top Down');
            xlabel('X'); ylabel('Y');

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
        [T, Ts]          = DH(obj, config)                       
        collision        = checkSelfCollision(obj, solutions) 
        withinLimits     = checkJointLimits(obj, solution) 


        reachable        = isReachable(obj, x,y,z,phi)   


        solutions        = findJointAngles(obj, x,y,z,phi) 
        validSolutions   = findValidSolution(obj, solutions) 
        bestSolution     = findSolution(obj, validSolutions)  

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
        end





        function ensurePlot(obj)
            if ~obj.isSimulated, return; end
            if isempty(obj.simFigure) || ~isvalid(obj.simFigure)
                obj.initPlot();
            end
        end

        function initPlot(obj)
            obj.simFigure = figure('Name', 'Arm Simulator', 'NumberTitle', 'off');

            obj.simAxes3D  = subplot(1,2,1, 'Parent', obj.simFigure);
            hold(obj.simAxes3D,  'on'); grid on; axis equal;
            view(obj.simAxes3D,  3);
            title(obj.simAxes3D, '3D View');
            xlabel('X'); ylabel('Y'); zlabel('Z');

            obj.simAxesTop = subplot(1,2,2, 'Parent', obj.simFigure);
            hold(obj.simAxesTop, 'on'); grid on; axis equal;
            view(obj.simAxesTop, 2);
            title(obj.simAxesTop, 'Top Down');
            xlabel('X'); ylabel('Y');

            obj.updatePlot();   % draw home position immediately
        end

        function updatePlot(obj)
            if ~obj.isSimulated || isempty(obj.simFigure) || ~isvalid(obj.simFigure)
                return
            end

            % Get transforms for current joint angles
            [~, Ts] = obj.DH();

            % Build positions: base (origin) + one point per joint
            % Row 1 = base, rows 2..N+1 = cumulative joint positions
            numJoints = length(Ts);
            positions = zeros(numJoints + 1, 3);
            positions(1,:) = [0, 0, 0];   % base
            for i = 1:numJoints
                positions(i+1,:) = Ts{i}(1:3, 4)';
            end

            endEffector = positions(end,:);

            % Accumulate end effector history
            obj.poseHistory = [obj.poseHistory; endEffector];

            % --- 3D view ---
            cla(obj.simAxes3D);

            % Draw arm links
            plot3(obj.simAxes3D, ...
                positions(:,1), positions(:,2), positions(:,3), ...
                'o-', 'LineWidth', 2, 'MarkerSize', 6, ...
                'Color', [0.2 0.4 0.8], 'MarkerFaceColor', [0.2 0.4 0.8]);

            % Highlight end effector
            plot3(obj.simAxes3D, endEffector(1), endEffector(2), endEffector(3), ...
                'r*', 'MarkerSize', 10);

            % End effector trace
            if size(obj.poseHistory, 1) > 1
                plot3(obj.simAxes3D, ...
                    obj.poseHistory(:,1), obj.poseHistory(:,2), obj.poseHistory(:,3), ...
                    '--', 'Color', [0.6 0.6 0.6]);
            end

            % Keep axes from resizing on every update
            axis(obj.simAxes3D, 'equal');

            % --- Top-down view ---
            cla(obj.simAxesTop);

            plot(obj.simAxesTop, ...
                positions(:,1), positions(:,2), ...
                'o-', 'LineWidth', 2, 'MarkerSize', 6, ...
                'Color', [0.2 0.4 0.8], 'MarkerFaceColor', [0.2 0.4 0.8]);

            plot(obj.simAxesTop, endEffector(1), endEffector(2), ...
                'r*', 'MarkerSize', 10);

            if size(obj.poseHistory, 1) > 1
                plot(obj.simAxesTop, ...
                    obj.poseHistory(:,1), obj.poseHistory(:,2), ...
                    '--', 'Color', [0.6 0.6 0.6]);
            end

            axis(obj.simAxesTop, 'equal');

            drawnow;
        end



    end
end
