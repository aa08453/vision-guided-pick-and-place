function visualize_3d_poses(pointCloud, instanceMap, stats, subplot_num)
    % Visualize 3D poses and coordinate frames for each object
    
    % subplot(3, 2, subplot_num);
    figure;
    pcshow(pointCloud, "VerticalAxisDir", "Up");
    hold on;
    axis equal;
    xlabel('X', 'FontSize', 11); 
    ylabel('Y', 'FontSize', 11); 
    zlabel('Z', 'FontSize', 11);
    title('3D Object Poses', 'FontSize', 12, 'FontWeight', 'bold');
    set(gca, 'Color', 'black');
    set(gcf, 'Color', 'black');
    
    % Plot pose for each object (skip ones with no valid 3D support)
    for k = 1:height(stats)
        pose = compute_object_pose(pointCloud, instanceMap, k);
        if ~isnan(pose.yaw)
            visualize_coordinate_frame(pose);
        end
    end
    
    set(gca, 'ZDir', 'reverse');
    xlim([-0.2, 0.2]);
    ylim([-0.3, 0.3]);
    zlim([0.4, 0.6]);
    hold off;
end
