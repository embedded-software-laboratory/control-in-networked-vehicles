function [] = main(varargin)
%MAIN A template function for a HLC
%   The input is expected to be a comma separated list of vehicle IDs.

%% Initialize data readers/writers...
common_cpm_functions_path = fullfile( ...
    '..','..','..','high_level_controller/examples/matlab' ...
);
assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
addpath(common_cpm_functions_path);

matlabDomainId = 1;
[matlabParticipant, reader_vehicleStateList, ~, writer_vehicleCommandPathTracking, reader_systemTrigger, writer_readyStatus, trigger_stop] = init_script(matlabDomainId);



% Add folder with common functions
common_functions_path = '../common';
assert(isfolder(common_functions_path));
addpath(common_functions_path);

% Controller setup
vehicle_ids = cell2mat(varargin);
Ts = 0.1;   % sample time [s]
vehicle_model = get_longitudinal_model(Ts);

% Reference path generation
path_points = get_path_points('outer_lane');

% Store data for visualization
yRefTotal = zeros(size(vehicle_model.C,1),0);
yTotal = zeros(size(vehicle_model.C,1),0);
uTotal = zeros(size(vehicle_model.B,2),0);
t = zeros(1,0);
% loop variables
loop_init = false;
y = zeros(size(vehicle_model.C,1),1);
yref = @(t) [t;0];
t_start_nanos = 0;


% Set reader properties
reader_vehicleStateList.WaitSet = true;
reader_vehicleStateList.WaitSetTimeout = 5; % [s]
% Middleware period for valid_after stamp
dt_period_nanos = uint64(Ts*1e9);

%% Sync start with infrastructure
% Send ready signal for all assigned vehicle ids
disp('Sending ready signal');
for i_vehicle = vehicle_ids
    ready_msg = ReadyStatus;
    ready_msg.source_id = strcat('hlc_', num2str(i_vehicle));
    ready_stamp = TimeStamp;
    ready_stamp.nanoseconds = uint64(0);
    ready_msg.next_start_stamp = ready_stamp;
    writer_readyStatus.write(ready_msg);
end
% Wait for start or stop signal
disp('Waiting for start or stop signal');    
got_stop = false;
got_start = false;
while (~got_stop && ~got_start)
    [got_start, got_stop] = read_system_trigger(reader_systemTrigger, trigger_stop);
end
%% Run the HLC
% Main control loop
while (~got_stop)
    % Measure system output
    % ---------------------
    [sample, ~, sample_count, ~] = reader_vehicleStateList.take();
    if (sample_count > 1)
        warning('Received %d samples, expected 1. Correct middleware period? Missed deadline?', sample_count);
    end
    y = y;
    % Compute control action
    % ----------------------
    if loop_init == false
        t_start_nanos = sample(end).t_now;
        loop_init = true;
    end
    t_rel = double(sample(end).t_now - t_start_nanos)*1e-9;
    u = u;
    
    % Store vaules
    yTotal(:,end+1) = y(1:2)';
    yRefTotal(:,end+1) = yref(t_rel);
    uTotal(:,end+1) = u;
    t(:,end+1) = t_rel;
    
    % Apply control action
    % --------------------
    vehicle_command_path_tracking = VehicleCommandPathTracking;
    vehicle_command_path_tracking.vehicle_id = uint8(vehicle_ids(1));
    vehicle_command_path_tracking.path = path_points;
    vehicle_command_path_tracking.speed = u;
    vehicle_command_path_tracking.header.create_stamp.nanoseconds = ...
        uint64(sample(end).t_now);
    vehicle_command_path_tracking.header.valid_after_stamp.nanoseconds = ...
        uint64(sample(end).t_now + dt_period_nanos);
    writer_vehicleCommandPathTracking.write(vehicle_command_path_tracking);
    
    % Check for stop signal
    [~, got_stop] = read_system_trigger(reader_systemTrigger, trigger_stop);
end


%% Visualization
% Distance
figure;
hold on
plot(t, yRefTotal(1,:));
plot(t, yTotal(1,:));
legend('s_{ref}','s')
title('Distance')
hold off;

% Velocity
figure
hold on
plot(t, v_min*ones(size(t)));
plot(t, v_max*ones(size(t)));
plot(t, uTotal(1,:));
plot(t, yTotal(2,:));
legend("v_{min}", "v_{max}", "v_{in}", "v")
title('Velocity')
hold off
end

