function s_k_on_loop = compute_distance_on_path(position, path_points)
%   COMPUTE_DISTANCE_ON_PATH Computes the distance along the path PATH_POINTS
%   Input is a POSITION [x, y] and a path PATH_POINTS
%   Output is a distance in the range of [0, path_points(end).s]

    assert(numel(path_points) > 1);

    num_subdivs = 4;

    n_points = numel(path_points);
    is = 1:n_points-1;
    
    % convert array of path point objects to array of structs
    path = struct([]);
    for i = 1:length(path_points)
        path(i).s = path_points(i).s;
        path(i).x = path_points(i).pose.x;
        path(i).y = path_points(i).pose.y;
        path(i).yaw = path_points(i).pose.yaw;
    end
    
    s_a = horzcat( path(is).s );
    s_b = horzcat( path(is+1).s );

    for iterations=1:5
        % create subsegments

        new_is = ones(1,numel(is) * num_subdivs);
        new_sa = ones(1,numel(is) * num_subdivs);
        new_sb = ones(1,numel(is) * num_subdivs);

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

        [dx, dy] = compute_distance_on_segment(position, s_query, path(is), path(is+1));
       
        distance_squared = dx .* dx + dy .* dy;

        numrows = numel(distance_squared);
        [~, ~, ~,localminidx] = cmmn.extrema([inf, distance_squared, inf]);
        localmin = false(1,numrows);
        localmin(localminidx-1) = true;

        is = is(localmin);
        s_a = s_a(localmin);
        s_b = s_b(localmin);
    end

    distance_squared = distance_squared(localmin);

    s_query = (s_a+s_b)/2;
    
    query_dist = [s_query; distance_squared];
    
    [~,idx] = sort(query_dist(2,:));
    sorted = query_dist(:,idx);
    s_k_on_loop = sorted(1,1);
end

function [dx, dy] = compute_distance_on_segment(position, s_query, point_a, point_b)
    interpolation = path_interpolation(s_query, point_a, point_b);
    
    dx = position(1)*ones(1,length(s_query)) - interpolation.position_x;
    dy = position(2)*ones(1,length(s_query)) - interpolation.position_y;
end

function interp = path_interpolation(s_queried, start_point, end_point)
    % convert to arrays
    s_start = horzcat( start_point.s );
    s_end = horzcat( end_point.s );
    position_start_x = horzcat( start_point.x );
    position_end_x = horzcat( end_point.x );
    position_start_y = horzcat( start_point.y );
    position_end_y = horzcat( end_point.y );
    position_start_yaw = horzcat( start_point.yaw );
    position_end_yaw = horzcat( end_point.yaw );

    delta_s = s_end - s_start;
    tau = (s_queried - s_start) ./ delta_s;
    
    tau2 = tau .* tau;
    tau3 = tau .* tau2;
    
    velocity_start_x = cos(position_start_yaw) .* delta_s;
    velocity_start_y = sin(position_start_yaw) .* delta_s;
    velocity_end_x = cos(position_end_yaw) .* delta_s;
    velocity_end_y = sin(position_end_yaw) .* delta_s;
    
    
    % Hermite spline coefficients
    p0 = 2*tau3 - 3*tau2 + 1;
    m0 = tau3 - 2*tau2 + tau;
    p1 = -2*tau3 + 3*tau2;
    m1 = tau3 - tau2;
    
    interp.position_x =  position_start_x .*   p0 + velocity_start_x .*   m0 + position_end_x .*   p1 + velocity_end_x .*   m1;
    interp.position_y =  position_start_y .*   p0 + velocity_start_y .*   m0 + position_end_y .*   p1 + velocity_end_y .*   m1;
end

