function s_k_on_loop = compute_distance_on_path(position, path_points)
    num_subdivs = 4;

    n_points = numel(path_points);
    is = 1:n_points-1;
    
    s_a = arrayfun(@(i) path_points(i).s, is);
    s_b = arrayfun(@(i) path_points(i).s, is + 1);
    
    for iterations=1:5
        % create subsegments
        
        new_is = ones(numel(is) * num_subdivs,1);
        new_sa = ones(numel(is) * num_subdivs,1);
        new_sb = ones(numel(is) * num_subdivs,1);
        
        for i=1:numel(is)
            delta_s = (s_b(i) - s_a(i)) / num_subdivs;
            s_base = s_a(i);
            for j=1:num_subdivs
                new_i = (i-1) * num_subdivs + j;
                new_is(new_i) = is(i);
                new_sa(new_i) = s_base;
                s_base = s_base + delta_s;
                new_sb(new_i) = s_base;
            end
        end
        
        
        
        is = new_is;
        s_a = new_sa;
        s_b = new_sb;

        s_query = (s_b + s_a) / 2;
        [dx, dy] = arrayfun(@(i, sq) compute_distance_on_segment(position, sq, path_points(i), path_points(i+1)), is, s_query);
        distance_squared = dx .* dx + dy .* dy;
        
        numrows = numel(distance_squared);
        localmin = islocalmin([inf, distance_squared.', inf]);
        localmin = localmin(2:numrows+1);

        is = is(localmin);
        s_a = s_a(localmin);
        s_b = s_b(localmin);
    end
    
    distance_squared = distance_squared(localmin);

    s_query = (s_a+s_b)/2;
    
    sorted = sortrows(table(s_query,distance_squared),2);
    s_k_on_loop = sorted{1,1};
end

function [dx, dy] = compute_distance_on_segment(position, s_query, point_a, point_b)
    interpolation = path_interpolation(s_query, point_a, point_b);
    
    dx = position(1) - interpolation.position_x;
    dy = position(2) - interpolation.position_y;
    
end