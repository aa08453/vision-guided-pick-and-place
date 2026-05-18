function validate_results(sorted, desired, frame_num)
    % VALIDATE_RESULTS Display validation results for a single frame/test
    %
    % Parameters:
    %   sorted    - String array of detected colors
    %   desired   - String array of expected colors
    %   frame_num - Frame or test number (for display)
    
    [is_pass, received_colors] = check_test_result(sorted, desired);
    
    if is_pass
        status = "✅";
    else
        status = "❌";
    end
    des_str = strjoin(desired, ", ");
    rec_str = strjoin(received_colors, ", ");
    
    fprintf('[Desired: %-20s] [Received: %-20s] %s\n', ...
        des_str, rec_str, status);
end
