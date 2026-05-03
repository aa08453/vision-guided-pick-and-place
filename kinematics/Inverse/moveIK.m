function [validSolutions, bestConfig, fig] = moveIK(arb,x,y,z,phi,handle, config)

if isnan(phi) && isnumeric(phi)
    phi = atan2(z-24.2, sqrt(x^2 + y^2));
    % phi = 0;
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
    [x_obtained,y_obtained,z_obtained, R_obtained] = pincherFK(configs(i,:));
    
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

if isnumeric(arb) && isnan(arb)

showdetails(robot)
figure(Name="Phantom X Pincher")
show(robot);

set(groot,'DefaultFigureWindowStyle','docked')


initialJoints = config;


[validSolutions, bestConfig] = findSolution(x,y,z,phi,robot,initialJoints);
disp(bestConfig);
disp(rad2deg(bestConfig))

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
% prevConfig = initialJoints;
% 
for k = 1:N
    cla;
    config_k = initialJoints + (bestConfig - initialJoints) * t(k); 
    show(robot, config_k, 'Collisions', 'on', 'Visuals', 'off');
    hold on;
    drawJointAxes(robot, config_k, 2);
    scatter3(x,y,z);
    % hold off;
    view(45, 30);
    zlim([0, 40]);
    drawnow;

end

% [x0, y0, z0, ~] = pincherFK(initialJoints);
% fprintf("Initial position: x=%.2f, y=%.2f, z=%.2f\n", x0, y0, z0);
% 
% figure(Name="Animating pincher movement");
% prevConfig = initialJoints;
% 
% for k = 1:N
    % x_k = x0 + t(k) * (x - x0);
    % y_k = y0 + t(k) * (y - y0);
    % z_k = z0 + t(k) * (z - z0);
    % 
    % if ~isReachable(x_k, y_k, z_k, phi)
    %     warning("Waypoint t=%.2f unreachable", t(k));
    %     continue;
    % end
    % 
    % configs_k = findJointAngles(x_k, y_k, z_k, phi);
    % 
    % if isempty(configs_k)
    %     warning("No IK solution at t=%.2f", t(k));
    %     continue;
    % end

    % Filter: z must stay above ground
    % validConfigs = [];
    % for i = 1:size(configs_k, 1)
    %     if ~checkJointLimits(configs_k(i,:))
    %         continue;
    %     end
    %     [~, ~, z_fk, ~] = pincherFK(configs_k(i,:));
    %     if z_fk >= -1e-3
    %         validConfigs = [validConfigs; configs_k(i,:)];
    %     end
    % end
    % 
    % if isempty(validConfigs)
    %     warning("No above-ground valid config at t=%.2f", t(k));
    %     continue;
    % end
    % 
    % % Pick closest config to previous (ensures smooth motion)
    % dists = vecnorm(validConfigs - prevConfig, 2, 2);
    % [~, bestIdx] = min(dists);
    % config_k = validConfigs(bestIdx, :);
    % prevConfig = config_k;
    % 
    % % Visualize (config_k is in DH space, correct for show/FK)
    % [x_, y_, z_, ~] = pincherFK(config_k);
    % disp("x = " + x_ + ", y = " + y_ + ", z = " + z_);

    % cla;
    % config_k = initialJoints + (bestConfig - initialJoints) * t(k); 
    % show(robot, config_k, 'Collisions', 'on', 'Visuals', 'off');
    % hold on;
    % drawJointAxes(robot, config_k, 2);
    % scatter3(x,y,z);
    % % hold off;
    % view(45, 30);
    % zlim([0, 40]);
    % drawnow;
% end


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
    setJoints(arb, sim2real(config_k), 80);
end

fig = NaN;

end
