function s_k_on_loop = compute_distance_on_path(position, path_points)
%   COMPUTE_DISTANCE_ON_PATH Computes the distance along the path PATH_POINTS
%   Input is a POSITION [x, y], a path PATH_POINTS, and a current distance S_IN
%   Output is a distance in the range of [0, Inf]

    assert(numel(path_points) > 1);

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

function interp = path_interpolation(s_queried, start_point, end_point)
    s_start = start_point.s;
    s_end = end_point.s;

    delta_s = s_end - s_start;
    tau = (s_queried - s_start) / delta_s;
    
    tau2 = tau * tau;
    tau3 = tau * tau2;
    
    position_start_x = start_point.pose.x;
    position_start_y = start_point.pose.y;
    position_end_x = end_point.pose.x;
    position_end_y = end_point.pose.y;
    
    velocity_start_x = cos(start_point.pose.yaw) * delta_s;
    velocity_start_y = sin(start_point.pose.yaw) * delta_s;
    velocity_end_x = cos(end_point.pose.yaw) * delta_s;
    velocity_end_y = sin(end_point.pose.yaw) * delta_s;
    
    
    % Hermite spline coefficients
    p0 = 2*tau3 - 3*tau2 + 1;
    m0 = tau3 - 2*tau2 + tau;
    p1 = -2*tau3 + 3*tau2;
    m1 = tau3 - tau2;
    
    interp.position_x =  position_start_x *   p0 + velocity_start_x *   m0 + position_end_x *   p1 + velocity_end_x *   m1;
    interp.position_y =  position_start_y *   p0 + velocity_start_y *   m0 + position_end_y *   p1 + velocity_end_y *   m1;
end

