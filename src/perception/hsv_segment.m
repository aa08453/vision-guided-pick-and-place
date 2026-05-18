function [mergedMasks, instanceMap, sortedLabels, idColors] = hsv_segment(rgb_img)
    % HSV_SEGMENT  Generic saturated-region segmentation (color-agnostic).
    %
    % Color logic is currently COMMENTED OUT. This function now treats every
    % sufficiently saturated/bright blob as a single class ("any"). The
    % per-color hue branches are preserved below as comments so we can
    % re-enable color later by swapping the active and disabled blocks.
    %
    % Pipeline:
    %   1) Saturation + Value threshold (any hue)
    %   2) Morphological cleanup (bwareaopen + imfill)
    %   3) Geometric filter on connected components (Area + Solidity)
    %   4) Each surviving connected component gets a unique instance ID.
    %
    % Inputs:
    %   rgb_img      - H x W x 3 uint8 RGB image
    %
    % Outputs:
    %   mergedMasks  - H x W x 4 logical. Layer 1 = all detections; layers
    %                  2..4 are empty (kept for back-compat with consumers
    %                  that index by color channel).
    %   instanceMap  - H x W double, unique integer ID per detected object
    %   sortedLabels - string column. Currently always ["any"] when any
    %                  cube is found.
    %   idColors     - string column, idColors(k) = "any" for every instance.

    I = rgb_img;
    hsvI = rgb2hsv(I);
    H = hsvI(:,:,1);                       % hue unused while color is off
    S = hsvI(:,:,2);
    V = hsvI(:,:,3);

    % Adaptive saturation floor: graythresh picks an Otsu-style cutoff on S,
    % but clamp to 0.3 so a desaturated background can't drag it down.
    satFloor = max(graythresh(S), 0.3);
    valFloor = 0.15;   % rejects deep shadows / near-black pixels

    [imH, imW, ~] = size(I);
    mergedMasks = false(imH, imW, 4);   % shape preserved; only layer 1 used
    instanceMap = zeros(imH, imW);
    currentID = 1;
    sortedLabels = strings(0, 1);
    idColors    = strings(0, 1);    % idColors(k) = color of instance with ID k

    % ---- COLOR-AGNOSTIC PASS (active) --------------------------------------
    % Anything saturated + bright enough qualifies; hue is ignored.
    mask = (S > satFloor) & (V > valFloor);
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
            idColors(currentID, 1) = "any"; %#ok<AGROW>
            currentID = currentID + 1;
        end
    end
    mergedMasks(:, :, 1) = layerMask;
    if any(layerMask, 'all')
        sortedLabels(end+1, 1) = "any";
    end

    % ---- COLOR-AWARE PASS (disabled; re-enable to bring colors back) -------
    labels = ["yellow"; "red"; "green"; "blue"];
    for c = 1:4
        switch c
            % case 1   % Yellow / orange
            %     hueMask = (H >= 0.08) & (H <= 0.18);
            case 1   % Red (wraps the 0/1 boundary)
                hueMask = (H <= 0.03) | (H >= 0.95);
            case 2   % Green / teal
                % Extended to ~0.52 so dark teal cubes (e.g. the reference
                % RGB(20,60,60) at H = 0.50) aren't lost in the gap to blue.
                hueMask = (H >= 0.22) & (H <= 0.5);
            case 3   % Blue
                % Starts just above green to avoid double-claiming teal pixels
                % (each cluster ends up in exactly one of the 4 layers).
                hueMask = (H > 0.5) & (H <= 0.70);
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
