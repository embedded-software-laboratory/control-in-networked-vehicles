function ds = compute_rel_distance_on_path(path_points, s_1, s_2)
%   COMPUTE_REL_DISTANCE_ON_PATH computes the distance from position s_1 to s_2 on the path 
    s_max = path_points(end).s;

    %% produce result
    ds = s_2 - s_1;
    % Check if start / end line was crossed
    if ((s_2-s_1) < -s_max/2)
        ds = ds + s_max;
    end
end