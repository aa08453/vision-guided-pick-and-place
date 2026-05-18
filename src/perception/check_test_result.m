function [is_pass, received_colors] = check_test_result(sorted, desired)
    % CHECK_TEST_RESULT Validate if detected colors match expected colors
    %
    % Parameters:
    %   sorted   - String array of detected colors from vision pipeline
    %   desired  - String array of expected colors
    %
    % Returns:
    %   is_pass        - Boolean indicating if test passed
    %   received_colors - Filtered string array (excluding background and grey)
    
    % Filter out background and grey colors
    keepIdx = (sorted ~= "background") & (sorted ~= "grey");
    received_colors = sorted(keepIdx);
    
    % Sort both arrays for comparison
    received_sorted = sort(strtrim(received_colors(:)));
    desired_sorted = sort(strtrim(desired(:)));
    
    % Check if they match
    is_pass = isequal(received_sorted, desired_sorted);
end