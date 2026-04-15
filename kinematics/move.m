clear; close all; clc;
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
robot.DataFormat = 'row';

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

lengths = [13.7, 10.5,10.5, 11];


for i = 1:numJoints
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};

    if i == 1
        % First link extends along Z
        % tform = trvec2tform([0, 0, lengths(i)/2]);

        tform = axang2tform([1 0 0 pi/2]) * trvec2tform([0, 0, lengths(i)/2]);

        addCollision(bodies{i}, "cylinder", [0.5, lengths(i)], tform);
        addBody(robot,bodies{i},"base")
    else
        % Remaining links extend along X
        tform = trvec2tform([-lengths(i)/2, 0, 0]) * axang2tform([0 1 0 pi/2]);
        addCollision(bodies{i}, "cylinder", [0.5, lengths(i)], tform);
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

set(groot,'DefaultFigureWindowStyle','docked')

x = 23;
y = 0;
z = 25;
phi = pi/4; % to determine phi from R or pass R




p_command = [x; y; z;];

Rz = @(a) [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1;];
Ry = @(a) [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a);];

configs = findJointAngles(x,y,z,phi);

figure(Name="All Configs");

tolerance = 1e-3;
num_correct = 0;
for i=1:size(configs,1)
    subplot(2,2,i);
    show(robot, configs(i,:), 'Collisions', 'on', 'Visuals', 'off');
    [x_obtained,y_obtained,z_obtained, R_obtained] = pincherFK(configs(i,:), 0);
    
    p_obtained = [x_obtained; y_obtained; z_obtained;];
    
    p_err = norm(p_obtained - p_command);
    
    theta1 = configs(i,1);
    Rz_inv = Rz(-theta1);
    R_planar = Rz_inv * R_obtained;
    phi_obtained = wrapToPi(atan2(R_planar(3,1), R_planar(1,1)));
    phi_err1 = wrapToPi(abs(angdiff(phi_obtained, phi)));
    phi_err2 = wrapToPi(abs(angdiff(phi_obtained, pi - phi)));

    % R_err = norm(R_obtained - R_command);

    % R error we need to derive from my phi somehow, or reduce R to phi.
    % Since R is the rotation from the fixed frame, there must be something
    % I can do with phi
    
    correct = 1;
    if (p_err >= tolerance)
        fprintf("Desired pos: %d %d %d, obtained pos: %d %d %d\n", x, y, z, x_obtained, y_obtained, z_obtained);
        correct = 0;
    end

    if ( (phi_err1 >= tolerance) && (phi_err2 >= tolerance))
        fprintf("Desired orientation: %f, obtained orientation: %f (offsets of pi are oky <3)\n", phi, phi_obtained);
        correct = 0;
    end
    num_correct = num_correct + correct;

    zlim([0,30])
    view(45, 30)
    title(sprintf('Config %d: θ1=%.2f θ2=%.2f θ3=%.2f θ4=%.2f', ...
    i, configs(i,1), configs(i,2), configs(i,3), configs(i,4)));
end

fprintf("Phi obtained: %f, phi commanded: %f", phi_obtained, phi);
if (num_correct == 4)
    fprintf("✅");
end
fprintf("\n%d/4 correct solutions\n", num_correct)




flag = "sim"; % or "sim"

if flag == "real"
    arb = Arbotix('port', 'COM4', 'nservos', 5);
    initialJoints = getCurrentPose(arb);   
elseif flag == "sim"
    initialJoints = [0 0 40 90];
end



bestConfig = findSolution(x,y,z,phi,robot,initialJoints);
disp(bestConfig);
figure(Name="Best Config");

subplot(1,2,1);
show(robot, initialJoints, 'Collisions', 'on', 'Visuals', 'off');
title(sprintf('Final: θ1=%.2f θ2=%.2f θ3=%.2f θ4=%.2f', ...
    initialJoints(1), initialJoints(2), initialJoints(3), initialJoints(4)));
zlim([0,30])
view(45, 30)
ax = gca;
axis tight;
ax.XLim = ax.XLim * 2;
ax.YLim = ax.YLim * 2;
ax.ZLim = ax.ZLim * 2;       


subplot(1,2,2);
show(robot, bestConfig, 'Collisions', 'on', 'Visuals', 'off');
title(sprintf('Final: θ1=%.2f θ2=%.2f θ3=%.2f θ4=%.2f', ...
    bestConfig(1), bestConfig(2), bestConfig(3), bestConfig(4)));
zlim([0,30])
view(45, 30)
ax = gca;
axis tight;
ax.XLim = ax.XLim * 2;
ax.YLim = ax.YLim * 2;
ax.ZLim = ax.ZLim * 2;      

bestConfig = dh2servo(bestConfig, false);

% if flag == "sim"
N = 20;
t = linspace(0, 1, N);
figure(Name="Animating pincher movement");
for k = 1:N
    % if (checkJointLimits(bestConfig) == true)
        config_k = initialJoints + t(k) * (bestConfig - initialJoints);
        show(robot, config_k, 'Collisions', 'on', 'Visuals', 'off');
        axis tight;
        view(45, 30);
        zlim([0, 40]);
        drawnow;
    % end
end
if flag == "real"
N = 20;
t = linspace(0, 1, N);
figure;
for k = 1:N
    % if (checkJointLimits(bestConfig) == true)
        config_k = initialJoints + t(k) * (bestConfig - initialJoints);
        c_config_k = [config_k 0];
        arb.setpos([1 2 3 4 5], c_config_k, 600);
        show(robot, config_k, 'Collisions', 'on', 'Visuals', 'off');
        axis tight;
        view(45, 30);
        zlim([0, 40]);
        drawnow;
    % end

end
end





% end