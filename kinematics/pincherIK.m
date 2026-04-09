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


set(groot,'DefaultFigureWindowStyle','docked')

x = 5;
y = 5;
z = 15;
phi = pi/3;
p_command = [x; y; z;]

Rz = @(a) [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1;];
Ry = @(a) [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a);];

configs = findJointAngles(x,y,z,phi);

figure;

tolerance = 1e-3;
num_correct = 0;
for i=1:length(configs)
    subplot(2,2,i);
    show(robot, convertForPlot(configs(i,:), robot ));
    [x_obtained,y_obtained,z_obtained, R_obtained] = pincherFK(configs(i,:), 0);
    
    p_obtained = [x_obtained; y_obtained; z_obtained;];
    
    p_err = norm(p_obtained - p_command);
    
    theta1 = configs(i,1);
    Rz_inv = Rz(-theta1);
    R_planar = Rz_inv * R_obtained;
    phi_obtained = atan2(R_planar(3,1), R_planar(1,1));
    phi_err = abs(wrapToPi(phi_obtained-phi));
    
    % R_err = norm(R_obtained - R_command);

    % R error we need to derive from my phi somehow, or reduce R to phi.
    % Since R is the rotation from the fixed frame, there must be something
    % I can do with phi
    
    correct = 1;
    if (p_err >= tolerance)
        
        fprintf("Desired pos: %d %d %d, obtained pos: %d %d %d\n", x, y, z, x_obtained, y_obtained, z_obtained);
        correct = 0;
    end

    if (phi_err >= tolerance)
        fprintf("Desired orientation: %f, obtained orientation: %f\n", phi, phi_obtained);
        correct = 0;
    end
    num_correct = num_correct + correct;

    zlim([0,30])
    view(45, 30)
    title(sprintf('Config %d: θ1=%.2f θ2=%.2f θ3=%.2f θ4=%.2f', ...
    i, configs(i,1), configs(i,2), configs(i,3), configs(i,4)));
end

fprintf("Phi obtained: %f, phi commanded: %f %\n", phi_obtained, phi);
fprintf("%d/4 correct solutions\n", num_correct)




%% Helpers
function [plottableConfig] = convertForPlot(config, robot)
    plottableConfig = homeConfiguration(robot);
    for i = 1:length(config)
        plottableConfig(i).JointPosition = config(i);
    end

end
