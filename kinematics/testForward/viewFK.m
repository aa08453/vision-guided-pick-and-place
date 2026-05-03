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

% Enter joint angles in the matrix below in radians
configDesired = [0,0,0,0];
configNow = (configDesired);

% Display robot in provided configuration
config = homeConfiguration(robot);
% show(robot,config);

set(groot,'DefaultFigureWindowStyle','docked')
% config = randomConfiguration(robot);
for i = 1:numJoints
    config(i).JointPosition = configNow(i);
end

show(robot,config);        



