function tests = test_compute_distance_on_path
    tests = functiontests(localfunctions);
end


function setupOnce(testCase)  % do not change function name
    % add DDS structure definitions to the search path
    common_cpm_functions_path = fullfile( ...
        getenv('HOME'), 'dev/software/cpm_lib/dds_idl_matlab' ...
    );

    assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
    addpath(common_cpm_functions_path);
end


% this test determines if compute_distance_on_path returns the correct
% s_query value when given an interpolated point
function test_reversibility(testCase)
    
    % Define reference trajectory
    map_center_x = 2.25;    % [m]
    map_center_y = 2.0;     % [m]
    path_px    = [     1,      0,     -1,      0,      1] + map_center_x;   % [m]
    path_py    = [     0,      1,      0,     -1,      0] + map_center_y;   % [m]
    path_yaw   = [1*pi/2, 2*pi/2, 3*pi/2,      0, 1*pi/2];                  % [rad]
    path_s     = [     0, 1*pi/2, 2*pi/2, 3*pi/2, 4*pi/2];                  % [m]
    
    % Create path
    path_points = [];
    for i = 1:numel(path_px)
        path_point = PathPoint;
        path_point.pose.x = path_px(i);
        path_point.pose.y = path_py(i);
        path_point.pose.yaw = path_yaw(i);
        path_point.s = path_s(i);

        path_points = [path_points, path_point];
    end
    
    % create interpolation
    s_query = 0.0;
    ds = 0.1;
    s_max = 4*pi/2;
    i_path_point = 1;
    s_in = [];
    s_out = [];
    
    while s_query < s_max
        %assert(i_path_point < size(path) - 1);
        if s_query > path_points(i_path_point+1).s
            i_path_point = i_path_point + 1;
        end
        
        if i_path_point > size(path_points)
            break
        end
        
        % calculate interpolation
        interp = path_interpolation(s_query, path_points(i_path_point), path_points(i_path_point + 1));
        
        % test function
        s_calculated = compute_distance_on_path([interp.position_x, interp.position_y], path_points);
        
        s_in = [s_in, s_query];
        s_out = [s_out, s_calculated];
        
        s_query = s_query + ds;
    end

    verifyEqual(testCase, s_out, s_in, 'RelTol', 1e-12);
end