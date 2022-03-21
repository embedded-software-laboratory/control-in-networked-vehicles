function [u, y,t] = read_input_output(varargin)
%READ_INPUT_OUTPUT  vehicle_id and takes recording path and
%                   returns three vectors with input, output and timestamp from the given dds recording
%                   If no recording is given, the function uses the latest one

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

assert(((nargin == 1) || (nargin == 2)), 'Incorrect number of arguments (1 or 2)');

% vehicle id from input argument
vehicle_id = varargin{1};

if nargin < 2
    % take latest folder
    recording_folders = dir('/tmp/cpm_lab_recordings/*');
    current_folder = recording_folders(end);
    assert(current_folder.isdir);
    current_recording = fullfile(...
    current_folder.folder, ...
    current_folder.name, ...
    'recording.dat' ...
    );
else
    % take given recording
    current_recording = varargin{2};
end

dds_domain = getenv("DDS_DOMAIN"); % use from environment variable

% get data from recording
dataByVehicle = preprocessing(dds_domain, current_recording);
state = dataByVehicle.state;
observation = dataByVehicle.observation;
pathtracking = dataByVehicle.pathtracking;
systemtrigger = dataByVehicle.systemtrigger;

% get necessary time stamps
p_t = pathtracking.valid_after_stamp;
o_t = observation.create_stamp;
s_t = state.create_stamp;
start_trigger = systemtrigger.systemtrigger_stamp(1);
% catch problem: recording only contains start trigger
try
stop_trigger = systemtrigger.systemtrigger_stamp(2);
catch
stop_trigger = inf;
end

% array filter based on start/stop trigger
p_filter = (p_t >= start_trigger) & (p_t <= stop_trigger);
o_filter = (o_t >= start_trigger) & (o_t <= stop_trigger);
s_filter = (s_t >= start_trigger) & (s_t <= stop_trigger);

% get only values in between start trigger and stop trigger
speed_in = pathtracking.speed(p_filter);
xs = observation.x(o_filter);
ys = observation.y(o_filter);
speed_out = state.speed(s_filter);

% drop time stamps before start trigger & after stop trigger
s_t(~s_filter) = [];
o_t(~o_filter) = [];
p_t(~p_filter) = [];
% merge time stamp lists
[t, ~, ~] = union(union(s_t, o_t), p_t);
% get position index of existing time stamps in merged time stamp list
[~, i_s, ~] = intersect(t, s_t);
[~, i_o, ~] = intersect(t, o_t);
[~, i_p, ~] = intersect(t, p_t);

% number of data point in output vector
n_data = length(t);

% initialize return vectors
u = inf(n_data, 1);
y = inf(n_data, 2);

% initialize MeasurementTransformer
mt = cmmn.MeasurementTransformer(pathtracking.path(1,:), [vehicle_id]);
% create pseudo vehicle state list
vehicle_state_list = VehicleStateList();
vehicle_state_list.state_list(1) = VehicleState();
vehicle_state_list.state_list(1).vehicle_id = vehicle_id;
vehicle_state_list.state_list(1).pose = Pose2D();

% fill return vectors with data
for i = 1:n_data
    % --- u (input speed) ---
    % index is below/above existing time stamps in pathtracking messages
    if i < i_p(1) || i > i_p(end)
        u(i) = 0;
    else
        % time stamp belonging to index exists in set of pathtracking messages
        if sum(i_p == i) == 1
            u_t = speed_in(i_p == i);
            u(i) = u_t;
        % use last valid input value
        else
            u(i) = u_t;
        end
    end
    
    
    % --- y_1 (distance) ---
    % index is below existing time stamps in observation messages
    if i < i_o(1)
        y(i,1) = 0;
    else
        % time stamp belonging to index exists in set of observation messages
        if sum(i_o == i) == 1
            % use observation data from ips
            vehicle_state_list.state_list(1).pose.x = xs(i_o == i);
            vehicle_state_list.state_list(1).pose.y = ys(i_o == i);
            % transform current position to distance
            y_t_1 = mt.measure_longitudinal(vehicle_state_list);
            y(i,1) = y_t_1(1);
        % use last valid input value
        else
            y(i,1) = y_t_1(1);
        end
    end
    
    % --- y_2 (output speed) ---
    % index is below/above existing time stamps in vehicle state messages
    if i < i_s(1) || i > i_s(end)
        y(i,2) = 0;
    else
        % time stamp belonging to index exists in set of vehicle state messages
        if sum(i_s == i) == 1
            y_t_2 = speed_out(i_s == i);
            y(i,2) = y_t_2;
        % use last valid output value
        else
            y(i,2) = y_t_2;
        end
    end
end

end

