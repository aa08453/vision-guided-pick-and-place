function visualize_step(image, title_str)
    % Helper function to display image in subplot
    % subplot(3, 2, subplot_num);
    figure;
    imshow(image);
    title(title_str, 'FontSize', 12, 'FontWeight', 'bold');
    set(gca, 'Color', 'black');
    set(gcf, 'Color', 'black');
end
