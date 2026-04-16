clear; close all; clc;


dhparams = [0   	pi/2	13.7   	0;
            10.5	0       0       0;
            10.5    0	    0	    0;
            11	    0	    0	    0];

numJoints = size(dhparams,1);

robot = rigidBodyTree;
robot.DataFormat = 'row';


bodies = cell(numJoints,1);
joints = cell(numJoints,1);

lengths = [13.7, 10.5,10.5, 11];


for i = 1:numJoints
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};

    if i == 1

        tform = axang2tform([1 0 0 pi/2]) * trvec2tform([0, 0, lengths(i)/2]);

        addCollision(bodies{i}, "cylinder", [0.5, lengths(i)], tform);
        addBody(robot,bodies{i},"base")
    else
        tform = trvec2tform([-lengths(i)/2, 0, 0]) * axang2tform([0 1 0 pi/2]);
        addCollision(bodies{i}, "cylinder", [0.5, lengths(i)], tform);
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end

showdetails(robot)
figure(Name="Phantom X Pincher")
show(robot);

set(groot,'DefaultFigureWindowStyle','docked')

x = -20;
y = -15;
z = 20;
% phi = 0; % to determine phi from R or pass R
phi = atan2(z, sqrt(x^2 + y^2));      % elevation angle to target
flag = "real"; % or "sim"
initialJoints = [0 0 0 0];
if flag == "real"
    arb = Arbotix('port', 'COM4', 'nservos', 5);
    zeroConfig(arb, 200);
    pause(3);
    initialJoints = getCurrentPose(arb);   
end

bestConfig = findSolution(x,y,z,phi,robot,initialJoints);
disp(bestConfig);
figure(Name="Best Config");

N = 20;
t = linspace(0, 1, N);
figure(Name="Animating pincher movement");
prevConfig = initialJoints;

for k = 1:N
    config_k = initialJoints + (initialJoints - bestConfig) * t(k); 
    show(robot, config_k, 'Collisions', 'on', 'Visuals', 'off');
    axis tight;
    view(45, 30);
    zlim([0, 40]);
    drawnow;

    if flag == "real"
        setJoints(arb, sim2real(config_k), 200);
    end
end
