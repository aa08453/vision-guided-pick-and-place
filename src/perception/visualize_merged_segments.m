 
function visualize_merged_segments(mergedMasks)
    % Visualize merged segments in 2D with custom colormap
    
    % Create weighted sum of masks
    weights = reshape([1, 2, 3, 4], [1, 1, 4]);
    mergedSegments = sum(double(mergedMasks) .* weights, 3);
    
    % Display weighted version
    % subplot(3, 2, subplot_2d);
    figure;
    % figure;
    imshow(mergedSegments);
    title('Segmented image', 'FontSize', 12, 'FontWeight', 'bold');
    set(gca, 'Color', 'black');
    
    % Define color map
    customColormap = [1 1 0;  % 1: Yellow
                      1 0 0;  % 2: Red
                      0 1 0;  % 3: Green
                      0 0 1]; % 4: Blue
    
    % Display colored version
    rgbLabelImage = label2rgb(mergedSegments, customColormap, 'w');  % White background
    % subplot(3, 2, subplot_3d);
    figure;
    imshow(rgbLabelImage);
    title('Segmented and merged image', 'FontSize', 12, 'FontWeight', 'bold');
    set(gca, 'Color', 'black');
end
