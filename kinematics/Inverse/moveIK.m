function [validSolutions, bestConfig, fig] = moveIK(arb,x,y,z,phi,handle, config)

if isnan(phi) && isnumeric(phi)
    phi = -atan2(z, sqrt(x^2 + y^2));
end

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

if isnumeric(arb) && isnan(arb)

showdetails(robot)
figure(Name="Phantom X Pincher")
show(robot);

set(groot,'DefaultFigureWindowStyle','docked')


initialJoints = config;


[validSolutions, bestConfig] = findSolution(x,y,z,phi,robot,initialJoints);
disp(bestConfig);

figure(Name="Best Config");
for i=1:size(validSolutions,1)
    subplot(2,2,i);
    show(robot, validSolutions(i,:), 'Collisions', 'on', 'Visuals', 'off');
    hold on; 
    scatter3(x,y,z);
    hold off;

end

figure(Name="Starting Ending");
subplot(2,1,1);
show(robot, initialJoints, 'Collisions', 'on', 'Visuals', 'off');
subplot(2,1,2);
show(robot, bestConfig, 'Collisions', 'on', 'Visuals', 'off');


N = 20;
t = linspace(0, 1, N);
if ~ishandle(handle)
    figure(Name="Animating pincher movement");
else
    figure(handle);
end
fig = gcf;
prevConfig = initialJoints;

for k = 1:N
    cla;
    config_k = initialJoints + (bestConfig - initialJoints) * t(k); 
    show(robot, config_k, 'Collisions', 'on', 'Visuals', 'off');
    hold on;
    scatter3(x,y,z);
    % hold off;
    view(45, 30);
    zlim([0, 40]);
    drawnow;

end

else



% arb = Arbotix('port', 'COM4', 'nservos', 5);
% zeroConfig(arb, 200);
% pause(2);
initialJoints = getCurrentPose(arb);   

[validSolutions, bestConfig] = findSolution(x,y,z,phi,robot,initialJoints);
disp(bestConfig);

N = 20;
t = linspace(0, 1, N);

for k = 1:N
    config_k = initialJoints + (bestConfig - initialJoints) * t(k); 
    setJoints(arb, sim2real(config_k), 200);
end

fig = NaN;

end
