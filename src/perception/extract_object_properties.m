function stats = extract_object_properties(instanceMap, area_threshold)
    % Extract properties of detected objects
    % Filters out small objects below area_threshold
    %
    % Parameters:
    %   instanceMap    - Instance label image where each object has unique ID
    %   area_threshold - Minimum area in pixels to keep object
    %
    % Returns:
    %   stats - Table with Area, MajorAxisLength, MinorAxisLength, Orientation, Centroid
    
    % Use instanceMap directly as label image (not binary)
    % This preserves individual object IDs
    allStats = regionprops("table", instanceMap, ...
        "Area", ...
        "MajorAxisLength", ...
        "MinorAxisLength", ...
        "Orientation", ...
        "Centroid");
    
    % Handle empty case
    if isempty(allStats)
        stats = table.empty(0, 5);
        stats.Properties.VariableNames = {'Area', 'MajorAxisLength', 'MinorAxisLength', 'Orientation', 'Centroid'};
        return;
    end
    
    % Filter by area threshold
    idx = [allStats.Area] > area_threshold;
    stats = allStats(idx, :);
    
    % If no objects pass threshold, return empty table
    if height(stats) == 0
        fprintf('Warning: No objects larger than %d pixels detected.\n', area_threshold);
    else
        fprintf('Detected %d objects above area threshold of %d pixels.\n', height(stats), area_threshold);
    end
end
