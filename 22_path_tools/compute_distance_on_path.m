function distance = compute_distance_on_path(position, path_points)
%COMPUTE_DISTANCE_ON_PATH Computes the distance along the path PATH
%   Input is a POSITION [x, y] and a PATH
%   Output is in the range of [s_min, s_max] of the path

    % Find closest point on path
    ds = 0.01;
    %assert(size(path) > 1);
    
    i_path_point = 1;
    s_max = path_points(end).s;
    min_dist = 1e300;
    s_query = 0;
    s_result = 0;
    
    while s_query < s_max
        %assert(i_path_point < size(path) - 1);
        if s_query > path_points(i_path_point+1).s
            i_path_point = i_path_point + 1;
        end
        
        if i_path_point >= size(path_points)
            break
        end
        
        % calculate distance to reference path
        interp = path_interpolation(s_query, path_points(i_path_point), path_points(i_path_point + 1));
        
        dx = position(1) - interp.position_x;
        dy = position(2) - interp.position_y;
        dist = sqrt(dx*dx + dy*dy);
        
        if dist < min_dist
            min_dist = dist;
            s_result = s_query;
        end
        
        s_query = s_query + ds;
    end
    
    distance = s_result;
    % Compute distance
end

