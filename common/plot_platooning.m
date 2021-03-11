function plot_platooning(vehicle_ids, dds_domain)
% add visualization and idl folders to path
visualization_path = fullfile( ...
    getenv('HOME'), 'dev/software/lab_control_center/recording/visualization' ...
);
idl_path = fullfile( ...
    getenv('HOME'), 'dev/software/cpm_lib/dds_idl_matlab' ...
);

assert(isfolder(visualization_path), 'Missing folder "%s".', visualization_path);
addpath(visualization_path);

assert(isfolder(idl_path), 'Missing folder "%s".', idl_path);
addpath(idl_path);


% TODO Die function soll ins lu_template
recording_folders = dir('/tmp/rti_recordings/*');
current_folder = recording_folders(end);
assert(current_folder.isdir);
current_recording = fullfile(...
    current_folder.folder, ...
    current_folder.name, ...
    'recording.dat' ...
);
dataByVehicle = preprocessing(dds_domain, current_recording);

nVeh = numel(vehicle_ids);


% determine t_min and t_max
vehicle_fieldname = ['vehicle_' int2str(vehicle_ids(1))];
t = dataByVehicle.(vehicle_fieldname).pathtracking.create_stamp;

t_min = t(1);
t_max = t(end);


close all;


%% t-v plot
% get normalized reference t values
vehicle_fieldname = ['vehicle_' int2str(vehicle_ids(1))];
t = dataByVehicle.(vehicle_fieldname).pathtracking.create_stamp - t_min;

% Leader reference velocity plot
t0 = 0;
t0 = t0 + 0;
t1 = t0 + 15;
t2 = t0 + 25;
t3 = t0 + 35;

v0ref = ...
    ((t>=t0) & (t<t1)) .* 0.5 ...
    + ((t>=t1) & (t<t2)) .* 1.4 ...
    + ((t>=t2) & (t<t3)) .* 0.8 ...
    + (t>=t3)            .* 0; ...
    

% setup figure
% set linewidth to 1
figure('position',[0 0 1920 997],'color',[1 1 1]);
title('Velocity over Time');
xlabel('Time $t\ [s]$','Interpreter','LaTex');
ylabel('Velocity $v\ [m/s]$','Interpreter','LaTex');
hold on;

set(0, 'DefaultLineLineWidth', 1);
    

% plot minimum and maximum velocities
v_min = 0.3;
v_max = 1.5;

plot(t, v_min*ones(size(t)), 'Color', [1, 0, 0]);
plot(t, v_max*ones(size(t)), 'Color', [1, 0, 0]);

plot(t,v0ref);

% array for plot legend
leg_a = cell(1,nVeh);

for iVeh = 1:nVeh
    vehicle_id = vehicle_ids(iVeh);
    leg_a{iVeh} = "v_{" + num2str(vehicle_id) + "}";
    vehicle_fieldname = ['vehicle_' int2str(vehicle_id)];
    t = dataByVehicle.(vehicle_fieldname).pathtracking.create_stamp - t_min;
    v = dataByVehicle.(vehicle_fieldname).pathtracking.speed;
    plot(t,v);
end

legend("v_{min}", "v_{max}", "v_{ref}", leg_a{:});
hold off;
xlim([0, t_max - t_min]);
ylim([0 v_max]);

% save as png
saveas(gcf,'t-v.png');



%% t-a plot
% get normalized reference t values
vehicle_fieldname = ['vehicle_' int2str(vehicle_ids(1))];
t = dataByVehicle.(vehicle_fieldname).pathtracking.create_stamp - t_min;

% setup figure
% set linewidth to 1
figure('position',[0 0 1920 997],'color',[1 1 1]);
title('Acceleration over Time');
xlabel('Time $t\ [s]$','Interpreter','LaTex');
ylabel('Acceleration $a\ [m/s^2]$','Interpreter','LaTex');
hold on;

set(0, 'DefaultLineLineWidth', 1);
    

% plot minimum and maximum velocities
a_min = -1;
a_max = 0.5;

plot(t, a_min*ones(size(t)), 'Color', [1, 0, 0]);
plot(t, a_max*ones(size(t)), 'Color', [1, 0, 0]);

% array for plot legend
leg_a = cell(1,nVeh);

for iVeh = 1:nVeh
    vehicle_id = vehicle_ids(iVeh);
    leg_a{iVeh} = "a_{" + num2str(vehicle_id) + "}";
    vehicle_fieldname = ['vehicle_' int2str(vehicle_id)];
    t = dataByVehicle.(vehicle_fieldname).pathtracking.create_stamp - t_min;
    v = dataByVehicle.(vehicle_fieldname).pathtracking.speed;
    dv = v(2:end) - v(1:end-1);
    dt = t(2:end) - t(1:end-1);
    %disp(size(dv));
    %disp(size(dt));
    a = dv ./ dt.';
    %disp(dv);
    plot(t(1:end-1),a);
end

legend("a_{min}", "a_{max}", leg_a{:});
hold off;
xlim([0, t_max - t_min]);
ylim([a_min a_max]);

% save as png
saveas(gcf,'t-a.png');



%% t-d plot
if nVeh>1
    
    vehicle_fieldname = ['vehicle_' int2str(vehicle_ids(1))];
    t = dataByVehicle.(vehicle_fieldname).state.create_stamp;
    t = t - t_min;
    t = t(t >= 0 & t <= t_max - t_min);
    
    
    vehPath = dataByVehicle.(vehicle_fieldname).pathtracking(1).path(1,:);
    
    ss = {};
    ts = {};
    
    for vehicle_id = vehicle_ids
        vehicle_fieldname = ['vehicle_' int2str(vehicle_id)];
        
        % get t, x, and y coordinates from vehicle state commands
        t = dataByVehicle.(vehicle_fieldname).state.create_stamp;
        x = [dataByVehicle.(vehicle_fieldname).state.x];
        y = [dataByVehicle.(vehicle_fieldname).state.y];

        % only use those positions that happened during the path tracking
        t = t - t_min;
        index = (t >= 0 & t <= t_max - t_min);
        t = t(index);
        x = x(index);
        y = y(index);
        
        % add computed loop positions to array
        ss = {ss{:}, arrayfun(@(x, y) compute_distance_on_path([x, y], vehPath), x, y)};
        ts = {ts{:}, t};
    end
    
    
    % setup plot
    figure('position',[0 0 1920 997],'color',[1 1 1]);
    title('Distance between pairs of vehicles over Time');
    xlabel('Time $t\ [s]$','Interpreter','LaTex');
    ylabel('Distance $d\ [m]$','Interpreter','LaTex');
    hold on;
    
    set(0, 'DefaultLineLineWidth', 1);
    
    
    % plot minimum and reference distances
    d_min = 0.3;
    d_ref = 0.5;
    
    plot(t, d_min*ones(size(t)), 'Color', [1, 0, 0]);
    plot(t, d_ref*ones(size(t)));
    
    
    % array for plot legend
    leg_act = cell(1,nVeh-1);
    
    for iVeh = 1:nVeh-1
        vehicle_id = vehicle_ids(iVeh);
        vehicle_fieldname = ['vehicle_' int2str(vehicle_id)];
        leg_act{iVeh} = "d_{" + num2str(vehicle_id)  + num2str(vehicle_id+1) + "}";
        
        % back vehicle
        s1 = ss{iVeh};
        t1 = ts{iVeh};
        
        % front vehicle
        s2 = ss{iVeh+1};
        t2 = ts{iVeh+1};
        
        % filter for timestamps shared by both vehicles path tracking
        % commands
        [t, i1, i2] = intersect(t1, t2);
        %disp([size(t); size(t1); size(t2)]);
        d = arrayfun(@(a, b) compute_rel_distance_on_path(vehPath, b, a), s1(i1), s2(i2)); 
        
        plot(t, d);
    end
    
    legend("d_{min}","d_{ref}", leg_act{:});
    hold off;
    xlim([0, t_max - t_min]);
    ylim([0, 2]);
    
    % save as png
    saveas(gcf,'t-d.png');
end



end


% computes the distance between two vehicles whose position on the path is
% known
function ds = compute_rel_distance_on_path(path_points, s_k_on_loop, s_k_minus_one_on_loop)
    s_max = path_points(end).s;
    
    %% produce result
    ds = s_k_on_loop - s_k_minus_one_on_loop;
    % Check if start / end line was crossed
    if ((s_k_on_loop-s_k_minus_one_on_loop) < -s_max/2)
        ds = ds + s_max;
    end
    
    %s_k = s_k_minus_one + ds;
    %if (s_k < -1e-15)
    %    warning("Distance traveled on path is %f < 0, setting to 0", s_k);
    %    s_k = 0;
    %end
end

