function plot_platooning(vehicle_ids)
% plot_platooning visualizes relevant data for the presentation of a platooning
%   controller or a one-vehicle controller using pathtracking commands

dds_domain = getenv('DDS_DOMAIN');
vehicle_ids = sort(vehicle_ids, 'descend');

% add visualization and idl folders to path
visualization_path = fullfile( ...
    '..','..','lab_control_center/recording/visualization' ...
);
idl_path = fullfile( ...
    '..','..','cpm_lib/dds_idl_matlab' ...
);

assert(isfolder(visualization_path), 'Missing folder "%s".', visualization_path);
addpath(visualization_path);

assert(isfolder(idl_path), 'Missing folder "%s".', idl_path);
addpath(idl_path);


recording_folders = dir('/tmp/cpm_lab_recordings/*');
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
vehicle_id = vehicle_ids(1);
if ~isfield(dataByVehicle(vehicle_id), 'pathtracking')
    warning("No pathtracking messages recorded, quitting plot")
    return
end
t = dataByVehicle(vehicle_id).pathtracking.create_stamp;

t_min = t(1);
t_max = t(end);


close all;

% Cost
dis_cost = 0;
vel_cost = 0;
pos_cost = 0;
q_vel = 1;
q_dis = 1;
q_pos = 1;


%% t-v_in plot
% get normalized reference t values
% TODO new preprocessing -- dataByVehicle is struct array
vehicle_id = vehicle_ids(1);
t = dataByVehicle(vehicle_id).pathtracking.create_stamp - t_min;

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
title('Input Velocity over Time','Interpreter','LaTex','FontSize',14);
xlabel('Time $t\ [s]$','Interpreter','LaTex');
ylabel('Input Velocity $v_{in}\ [m/s]$','Interpreter','LaTex');
hold on;

set(0, 'DefaultLineLineWidth', 1);

% array for plot legend
leg_a = cell(1,nVeh);

for iVeh = 1:nVeh
    leg_a{iVeh} = "v_{" + num2str(iVeh) + "}";
    vehicle_id = vehicle_ids(iVeh);
    t_i = dataByVehicle(vehicle_id).pathtracking.create_stamp - t_min;
    v_i = dataByVehicle(vehicle_id).pathtracking.speed;
    plot(t_i,v_i);
end

% plot minimum and maximum velocities
v_min = 0;
v_max = 1.5;

plot(t, v_min*ones(size(t)), 'Color', [1, 0, 0]);
plot(t, v_max*ones(size(t)), 'Color', [1, 0, 0]);

if nVeh > 1
    plot(t,v0ref, 'Color', [0.6, 0.6, 0.6]);
    legend(leg_a{:}, "v_{min}", "v_{max}", "v_{ref}");
else
    legend(leg_a{:}, "v_{min}", "v_{max}");
end



hold off;
xlim([0, t_max - t_min]);
% ylim([0 v_max]);

% save as png
saveas(gcf,'t-v.png');

%% t-v plot (output velocity)

% Leader reference velocity
t0 = 0;
t0 = t0 + 0;
t1 = t0 + 15;
t2 = t0 + 25;
t3 = t0 + 35;
v0ref = @(t)...
    ((t>=t0) & (t<t1)) .* 0.5 ...
    + ((t>=t1) & (t<t2)) .* 1.4 ...
    + ((t>=t2) & (t<t3)) .* 0.8 ...
    + (t>=t3)            .* 0; ...
    
% setup figure
% set linewidth to 1
figure('position',[0 0 1920 997],'color',[1 1 1]);
hold on;

set(0, 'DefaultLineLineWidth', 1);
    
% array for plot legend
leg_a = cell(1,nVeh);

for iVeh = 1:nVeh
    leg_a{iVeh} = "v_{" + num2str(iVeh) + "}";
    vehicle_id = vehicle_ids(iVeh);
    % only use those velocities that happened during the path tracking
    t = dataByVehicle(vehicle_id).state.create_stamp;
    v = dataByVehicle(vehicle_id).state.speed;
    t = t - t_min;
    index = (t >= 0 & t <= t_max - t_min);
    t = t(index);
    v = v(index);
    plot(t,v);
    vel_cost = vel_cost + q_vel*(v-v0ref(t))'*(v-v0ref(t));
end

% plot minimum and maximum velocities
v_min = 0;
v_max = 1.5;
plot(t, v_min*ones(size(t)), 'Color', [1, 0, 0]);
plot(t, v_max*ones(size(t)), 'Color', [1, 0, 0]);

if nVeh > 1
    % Leader reference velocity plot
    plot(t,v0ref(t), 'Color', [0.6, 0.6, 0.6]);
end

if nVeh > 1
    title_str = sprintf("Output Velocity over Time, $J_v=%.5g$", vel_cost);
    legend(leg_a{:}, "v_{min}", "v_{max}", "v_{ref}");
else
    title_str = sprintf("Output Velocity over Time");
    legend(leg_a{:}, "v_{min}", "v_{max}");
end
title(title_str,'Interpreter','LaTex','FontSize',14);
xlabel('Time $t\ [s]$','Interpreter','LaTex');
ylabel('Output Velocity $v\ [m/s]$','Interpreter','LaTex');
hold off;
xlim([0, t_max - t_min]);
% ylim([0 v_max] + 0.1*[-1 1]);

% save as png
saveas(gcf,'t-v_out.png');


%% t-a plot
% get normalized reference t values
vehicle_id = vehicle_ids(1);
t = dataByVehicle(vehicle_id).pathtracking.create_stamp - t_min;

% setup figure
% set linewidth to 1
figure('position',[0 0 1920 997],'color',[1 1 1]);
title('Input Acceleration over Time','Interpreter','LaTex','FontSize',14);
xlabel('Time $t\ [s]$','Interpreter','LaTex');
ylabel('Input acceleration $a_{in}\ [m/s^2]$','Interpreter','LaTex');
hold on;

set(0, 'DefaultLineLineWidth', 1);
    


% array for plot legend
leg_a = cell(1,nVeh);

for iVeh = 1:nVeh
    leg_a{iVeh} = "a_{" + num2str(iVeh) + "}";
    vehicle_id = vehicle_ids(iVeh);
    t = dataByVehicle(vehicle_id).pathtracking.create_stamp - t_min;
    v = dataByVehicle(vehicle_id).pathtracking.speed;
    dv = v(2:end) - v(1:end-1);
    dt = t(2:end) - t(1:end-1);
    %disp(size(dv));
    %disp(size(dt));
    a = dv ./ dt.';
    %disp(dv);
    plot(t(1:end-1),a);
end

% plot minimum and maximum velocities
a_min = -1;
a_max = 0.5;

plot(t, a_min*ones(size(t)), 'Color', [1, 0, 0]);
plot(t, a_max*ones(size(t)), 'Color', [1, 0, 0]);
legend(leg_a{:}, "a_{min}", "a_{max}");
hold off;
xlim([0, t_max - t_min]);
% ylim([a_min a_max]);

% save as png
saveas(gcf,'t-a.png');



%% t-d plot
if nVeh>1
    d_min = 0.3;
    d_ref = 0.5;
    
    vehicle_id = vehicle_ids(1);
    t = dataByVehicle(vehicle_id).state.create_stamp;
    t = t - t_min;
    t = t(t >= 0 & t <= t_max - t_min);
    
    
    vehPath = dataByVehicle(vehicle_id).pathtracking(1).path(1,:);
    
    ss = {};
    ts = {};
    
    for vehicle_id = vehicle_ids
        
        % get t, x, and y coordinates from vehicle state commands
        t = dataByVehicle(vehicle_id).state.create_stamp;
        x = [dataByVehicle(vehicle_id).state.x];
        y = [dataByVehicle(vehicle_id).state.y];

        % only use those positions that happened during the path tracking
        t = t - t_min;
        index = (t >= 0 & t <= t_max - t_min);
        t = t(index);
        x = x(index);
        y = y(index);
        
        % add computed loop positions to array
        ss = {ss{:}, arrayfun(@(x, y) cmmn.compute_distance_on_path([x, y], vehPath), x, y)};
        ts = {ts{:}, t};
    end
    
    
    % setup plot
    figure('position',[0 0 1920 997],'color',[1 1 1]);
    hold on;    
    set(0, 'DefaultLineLineWidth', 1);
    
    
    % array for plot legend
    leg_act = cell(1,nVeh-1);
    
    for iVeh = 1:nVeh-1
        leg_act{iVeh} = "d_{" + num2str(iVeh)  + num2str(iVeh+1) + "}";
        
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
        d = arrayfun(@(a, b) cmmn.compute_rel_distance_on_path(vehPath, b, a), s1(i1), s2(i2)); 
        
        plot(t, d);
        dis_cost = dis_cost + q_dis*(d-d_ref)'*(d-d_ref);
    end
    
    % plot minimum and reference distances    
    plot(t, d_min*ones(size(t)), 'Color', [1, 0, 0]);
    plot(t, d_ref*ones(size(t)), 'Color', [0.6, 0.6, 0.6]);
    
    legend(leg_act{:},"d_{min}","d_{ref}");
    hold off;
    xlim([0, t_max - t_min]);
    
    title_str = sprintf("Distance between pairs of vehicles over time, $J_d=%.5g$, $J=%.5g$", dis_cost,dis_cost+vel_cost);
    title(title_str,'Interpreter','LaTex','FontSize',14);
    xlabel('Time $t\ [s]$','Interpreter','LaTex');
    ylabel('Distance $d\ [m]$','Interpreter','LaTex');

    % save as png
    saveas(gcf,'t-d.png');
else
    % only one vehicle
    vehicle_id = vehicle_ids(1);
    
    % vehicle reference path
    vehPath = dataByVehicle(vehicle_id).pathtracking(1).path(1,:);

    % get t, x, and y coordinates from vehicle state commands
    t = dataByVehicle(vehicle_id).state.create_stamp;
    x = [dataByVehicle(vehicle_id).state.x];
    y = [dataByVehicle(vehicle_id).state.y];

    % only use those positions that happened during the path tracking
    t = t - t_min;
    index = (t >= 0 & t <= t_max - t_min);
    t = t(index);
    x = x(index);
    y = y(index);
    
    s = zeros(length(t),1);
    
    % initialize MeasurementTransformer
    mt = cmmn.MeasurementTransformer(vehPath, [vehicle_id]);
    
    % create pseudo vehicle state list
    vehicle_state_list = VehicleStateList();
    vehicle_state_list.state_list(1) = VehicleState();
    vehicle_state_list.state_list(1).vehicle_id = vehicle_id;
    vehicle_state_list.state_list(1).pose = Pose2D();
    
    for i = 1:length(t)
        % transform current position to distance
        vehicle_state_list.state_list(1).pose.x = x(i);
        vehicle_state_list.state_list(1).pose.y = y(i);
        % transform current position to distance
        s_t = mt.measure_longitudinal(vehicle_state_list);
        s(i) = s_t(1);
    end
    
    % calculate reference position on path
    y_ref = @(t) [s(1)+1.1*t - 0.5*sin(t)];
    s_ref = y_ref(t);
    
    % calculate position cost
    s_diff = arrayfun(@(s_1, s_2) cmmn.compute_rel_distance_on_path(vehPath, s_1, s_2), s, s_ref);
    pos_cost = q_pos*(s_diff)'*(s_diff);
    
    % setup plot
    figure('position',[0 0 1920 997],'color',[1 1 1]);
    hold on;    
    set(0, 'DefaultLineLineWidth', 1);
    
    % plot actual position
    plot(t, s);
    % and reference position over time
    plot(t, s_ref, 'Color', [0.6, 0.6, 0.6]);
    
    legend("s_{"+vehicle_id+"}","s_{ref}");
    hold off;
    xlim([0, t_max - t_min]);
    
    title_str = sprintf("Position of vehicle over time, $J_p=%.5g$", pos_cost);
    title(title_str,'Interpreter','LaTex','FontSize',14);
    xlabel('Time $t\ [s]$','Interpreter','LaTex');
    ylabel('Position $s\ [m]$','Interpreter','LaTex');

    % save as png
    saveas(gcf,'t-s.png');
end

end