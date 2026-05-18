function visualize_coordinate_frame(pose)
    % Visualize 3D coordinate frame for a given pose
    
    p = pose.d;  % Position [X Y Z]
    L = 0.05;    % Axis length
    
    % Plot origin
    plot3(p(1), p(2), p(3), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Compute axis endpoints
    x_axis = pose.R * [L; 0; 0];
    y_axis = pose.R * [0; L; 0];
    z_axis = pose.R * [0; 0; L];
    
    % Plot axes
    quiver3(p(1), p(2), p(3), x_axis(1), x_axis(2), x_axis(3), ...
        'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(p(1), p(2), p(3), y_axis(1), y_axis(2), y_axis(3), ...
        'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(p(1), p(2), p(3), z_axis(1), z_axis(2), z_axis(3), ...
        'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
end