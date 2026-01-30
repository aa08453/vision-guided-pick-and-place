function pixel_labels = segment_cluster(image_rgb, numColors, debug)
    arguments 
        image_rgb (:,:,:) double;
        numColors int32 = 3;
        debug logical = 0;
    end

    if (debug) 
        imshow(image_rgb);
    end

    imgLab = rgb2lab(image_rgb);
    ab = im2single(imgLab(:,:,2:3));
    pixel_labels = imsegkmeans(ab, numColors, NumAttempts=3);

    if (debug)
        [L, a, b] = imsplit(imgLab);
        imshow(L, []);
        histogram(L);
        imshow(a, []);
        histogram(a)
        imshow(b, []);
        histogram(b);
    end
end