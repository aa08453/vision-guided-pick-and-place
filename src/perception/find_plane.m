function [sliced_rgb, model] = find_plane(pointCloud)
    % figure;
    % imshow(rgb, [])
    % title("RGB Image")
    
    maxDistance = 0.01; % 2 cm
    referenceVector = [0, 0, 1]; % z axis
    maxAngularDistance = 5; % degrees
    
    [model,~,outlierIndices] = pcfitplane(pointCloud,...
                maxDistance,referenceVector,maxAngularDistance);
    
    % figure;
    
    sliced_rgb = uint8(zeros(480,640, 3));
    [row, col] = ind2sub([480 640], outlierIndices);
    for k = 1:length(outlierIndices)
        sliced_rgb(row(k), col(k), :) = uint8(pointCloud.Color(row(k), col(k), :));
    end

    
end