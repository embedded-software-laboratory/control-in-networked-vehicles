function [s_k_on_loop] = compute_distance_on_path(position, path_points)
%COMPUTE_DISTANCE_ON_PATH Computes the distance along the path PATH_POINTS
%   Input is a POSITION [x, y] and a path PATH_POINTS
%   Output is a distance in the range of [0, s_max]

    assert(numel(path_points) > 1);

    ds = 0.01;
    i_path_point = 1;
    s_max = path_points(end).s;
    min_dist = 1e300;
    s_query = 0;
    s_k_on_loop = 0;

    while s_query < s_max
        % Increase path point index if past a path point
        if (s_query > path_points(i_path_point+1).s)
            i_path_point = i_path_point + 1;
        end
        
        % Break loop if behind last path point
        if (i_path_point >= size(path_points))
            break
        end
        
        % calculate distance to reference path on current path segment
        interpolation = path_interpolation(s_query, path_points(i_path_point), path_points(i_path_point + 1));
        
        dx = position(1) - interpolation.position_x;
        dy = position(2) - interpolation.position_y;
        dist_to_path = sqrt(dx*dx + dy*dy);
        
        % update minimal distance and corresponding distance along path
        if (dist_to_path < min_dist)
            min_dist = dist_to_path;
            s_k_on_loop = s_query;
        end
        
        % increase path query distance
        s_query = s_query + ds;
    end
end

