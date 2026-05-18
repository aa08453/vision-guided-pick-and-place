function [mergedMasks, instanceMap, sortedLabels, idColors] = hsv_segment(rgb_img)
    % HSV_SEGMENT  Per-color HSV thresholding + geometric filtering.
    %
    % Replaces the K-means + reference-color matching + watershed pipeline
    % with a more robust approach inspired by Intro-To-Robotics
    % perception_pipeline_3d_regionprops.m:
    %   1) HSV thresholds per color (hue band + adaptive saturation + value)
    %   2) Morphological cleanup (bwareaopen + imfill)
    %   3) Geometric filter on connected components (Area + Solidity)
    %   4) Each surviving connected component gets a unique instance ID
    %      directly -- no watershed needed for separation.
    %
    % Inputs:
    %   rgb_img      - H x W x 3 uint8 RGB image
    %
    % Outputs:
    %   mergedMasks  - H x W x 4 logical, layers = [Yellow, Red, Green, Blue]
    %   instanceMap  - H x W double, unique integer ID per detected object
    %   sortedLabels - string column of colors that produced >=1 instance

    I = rgb_img;
    hsvI = rgb2hsv(I);
    H = hsvI(:,:,1);
    S = hsvI(:,:,2);
    V = hsvI(:,:,3);

    % Adaptive saturation floor: graythresh picks an Otsu-style cutoff on S,
    % but clamp to 0.25 so a desaturated background can't drag it down.
    satFloor = max(graythresh(S), 0.3);
    valFloor = 0.15;   % rejects deep shadows / near-black pixels

    labels = ["yellow"; "red"; "green"; "blue"];
    [imH, imW, ~] = size(I);
    mergedMasks = false(imH, imW, 4);
    instanceMap = zeros(imH, imW);
    currentID = 1;
    sortedLabels = strings(0, 1);
    idColors    = strings(0, 1);    % idColors(k) = color of instance with ID k

    for c = 1:4
        switch c
            case 1   % Yellow / orange
                hueMask = (H >= 0.06) & (H <= 0.18);
            case 2   % Red (wraps the 0/1 boundary)
                hueMask = (H <= 0.04) | (H >= 0.95);
            case 3   % Green / teal
                % Extended to ~0.52 so dark teal cubes (e.g. the reference
                % RGB(20,60,60) at H = 0.50) aren't lost in the gap to blue.
                hueMask = (H >= 0.22) & (H <= 0.52);
            case 4   % Blue
                % Starts just above green to avoid double-claiming teal pixels
                % (each cluster ends up in exactly one of the 4 layers).
                hueMask = (H > 0.55) & (H <= 0.70);
        end

        mask = hueMask & (S > satFloor) & (V > valFloor);
        mask = bwareaopen(mask, 100);
        mask = imfill(mask, 'holes');

        cc = bwconncomp(mask);
        cstats = regionprops(cc, 'Area', 'Solidity', 'PixelIdxList');

        layerMask = false(imH, imW);
        for j = 1:numel(cstats)
            if cstats(j).Area > 300 && cstats(j).Area < 15000 ...
                    && cstats(j).Solidity > 0.80
                layerMask(cstats(j).PixelIdxList) = true;
                instanceMap(cstats(j).PixelIdxList) = currentID;
                idColors(currentID, 1) = labels(c); %#ok<AGROW>
                currentID = currentID + 1;
            end
        end

        mergedMasks(:, :, c) = layerMask;
        if any(layerMask, 'all')
            sortedLabels(end+1, 1) = labels(c); %#ok<AGROW>
        end
    end
end
