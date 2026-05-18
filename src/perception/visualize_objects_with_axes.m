function visualize_objects_with_axes(instanceMap, stats)
    % Visualize objects with major and minor axes
    
    % subplot(3, 2, subplot_num);
    figure;
    imshow(label2rgb(instanceMap));  % White background
    hold on;
    title('Object Detection with Axes', 'FontSize', 12, 'FontWeight', 'bold');
    set(gca, 'Color', 'black');
    
    % Plot axes for each object
    for k = 1:height(stats)
        plot_object_axes(stats(k, :));
    end
    
    hold off;
end
 
