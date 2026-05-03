clc; close all; clear;

% This is a script to build the Phantom X Pincher in MATLAB based on its DH
% parameters. It is based on MATLAB example available at 
% https://www.mathworks.com/help/robotics/ug/build-manipulator-robot-using-kinematic-dh-parameters.html
% MATLAB creates a rigid body tree 


% Provide the DH parameters for the robot. The parameters are arranged in
% the order [a, alpha, d, theta], and going from link 1 to link n. The
% entry in the matrix corresponding to the joint variable is ignored. 
dhparams = [0   	pi/2	13.7   	0;
            10.5	0       0       0;
            10.5    0	    0	    0;
            11	    0	    0	    0];

numJoints = size(dhparams,1);

% Create a rigid body tree object.
robot = rigidBodyTree;

% Create a model of the robot using DH parameters.
% Create a cell array for the rigid body object, and another for the joint 
% objects. Iterate through the DH parameters performing this process:
% 1. Create a rigidBody object with a unique name.
% 2. Create and name a revolute rigidBodyJoint object.
% 3. Use setFixedTransform to specify the body-to-body transformation of the 
%    joint using DH parameters.
% 4. Use addBody to attach the body to the rigid body tree.
bodies = cell(numJoints,1);
joints = cell(numJoints,1);
for i = 1:numJoints
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};
    if i == 1 % Add first body to base
        addBody(robot,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end

% Verify that your robot has been built properly by using the showdetails or
% show function. The showdetails function lists all the bodies of the robot 
% in the MATLAB® command window. The show function displays the robot with 
% a specified configuration (home by default).
showdetails(robot)
figure(Name="Phantom X Pincher")
show(robot);

%% Forward Kinematics for different configurations
% Enter joint angles in the matrix below in radians
% configNow = [pi/3,pi/3,pi/3,pi/3];

% Display robot in provided configuration
% homeConfig = homeConfiguration(robot);
% show(robot,config);

numTests = 200;

numRight = 0;

set(groot,'DefaultFigureWindowStyle','docked')
for i = 1:numTests
    config = randomConfiguration(robot);
    configRaw = zeros(numJoints);
    for i = 1:numJoints
        configRaw(i) = config(i).JointPosition;
        % show(robot,config);        
    end

    goldenPose = getTransform(robot,config,"body4");
    [x,y,z,R] = pincherFK(configRaw, 0);
    testPose = [R, zeros(3,1);
                zeros(1,3), 1;];

    tol = 1e-6;

    if (norm(goldenPose - testPose) <= tol)
        figure('Position', 'WindowStyle', 'docked');
        disp("The golden pose is")
        disp(goldenPose)
        disp("The calculated pose is")
        disp(testPose)
        % show(robot,config);
        title("The golden configuration")
    end

    % Determine the pose of end-effector in provided configuration
    numRight = numRight + 1;
end

fprintf("Working with %d/%d\n", numRight, numTests);