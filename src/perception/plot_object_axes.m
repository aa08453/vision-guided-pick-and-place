function plot_object_axes(stat_row)
    % Plot major axis, minor axis, and centroid for a single object
    
    beam = stat_row.MajorAxisLength / 2;
    crossbeam = stat_row.MinorAxisLength / 2;
    avg = stat_row.Centroid;
    angle = stat_row.Orientation;  % In degrees
    
    % Convert angle to radians
    cosPhi = cosd(angle);
    sinPhi = sind(angle);
    
    % Major axis endpoints
    x1 = avg(1) - beam * cosPhi;
    y1 = avg(2) + beam * sinPhi;
    x2 = avg(1) + beam * cosPhi;
    y2 = avg(2) - beam * sinPhi;
    
    % Minor axis endpoints
    x3 = avg(1) + crossbeam * sinPhi;
    y3 = avg(2) + crossbeam * cosPhi;
    x4 = avg(1) - crossbeam * sinPhi;
    y4 = avg(2) - crossbeam * cosPhi;
    
    % Plot axes
    plot([x1, x2], [y1, y2], 'r', 'LineWidth', 2);      % Major axis (Red)
    plot([x3, x4], [y3, y4], 'b', 'LineWidth', 2);      % Minor axis (Blue)
    plot(avg(1), avg(2), 'y+', 'MarkerSize', 10, 'LineWidth', 2);  % Centroid (Yellow)
end
