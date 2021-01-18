function path_points = get_path_points(id)
path_points = struct([]);
switch id
    case 'circle'
        
        % Define reference trajectory
        map_center_x = 2.25;    % [m]
        map_center_y = 2.0;     % [m]
        x     = [     1,      0,     -1,      0,      1] + map_center_x;   % [m]
        y     = [     0,      1,      0,     -1,      0] + map_center_y;   % [m]
        yaw   = [1*pi/2, 2*pi/2, 3*pi/2,      0, 1*pi/2];                  % [rad]
        s     = [     0, 1*pi/2, 2*pi/2, 3*pi/2, 4*pi/2];                  % [m]
    case 'outer_lane'
        % Start on the top left with the straight, cw direction
        center_left_x = 1.25;
        center_right_x = 3.25;
        center_top_y = 2.75;
        center_bottom_y = 1.25;
        radius = 1;
        
        ds_lr = center_right_x - center_left_x;
        ds_tb = center_top_y - center_bottom_y;
        ds_qc = radius * pi/2;

        yaw = [ ...
            0; ...
            0; ...
            3*pi/2; ...
            3*pi/2; ...
            pi; ...
            pi; ...
            pi/2; ...
            pi/2; ...
        ];
        yaw(end+1) = yaw(1);

        x = [ ...
            center_left_x; ...
            center_right_x; ...
            center_right_x; ...
            center_right_x; ...
            center_right_x; ...
            center_left_x; ...
            center_left_x; ...
            center_left_x; ...
        ];
        x(end+1) = x(1);
        x = x - radius * sin(yaw);

        y = [ ...
            center_top_y; ...
            center_top_y; ...
            center_top_y; ...
            center_bottom_y; ...
            center_bottom_y; ...
            center_bottom_y; ...
            center_bottom_y; ...
            center_top_y; ...
        ];
        y(end+1) = y(1);
        y = y + radius * cos(yaw);

        s = [ ...
            0*ds_lr + 0*ds_qc + 0*ds_tb; ...
            1*ds_lr + 0*ds_qc + 0*ds_tb; ...
            1*ds_lr + 1*ds_qc + 0*ds_tb; ...
            1*ds_lr + 1*ds_qc + 1*ds_tb; ...
            1*ds_lr + 2*ds_qc + 1*ds_tb; ...
            2*ds_lr + 2*ds_qc + 1*ds_tb; ...
            2*ds_lr + 3*ds_qc + 1*ds_tb; ...
            2*ds_lr + 3*ds_qc + 2*ds_tb; ...
            2*ds_lr + 4*ds_qc + 2*ds_tb; ...
        ];
    otherwise
        warning('Unexpected ID, leaving path empty')
        x = [];
        y = [];
        yaw = [];
        s = [];
end

assert(numel(s) == numel(x));
assert(numel(s) == numel(y));
assert(numel(s) == numel(yaw));
for i = 1:numel(s)
    path_points(i).pose.x = x(i);
    path_points(i).pose.y = y(i);
    path_points(i).pose.yaw = yaw(i);
    path_points(i).s = s(i);
end

end