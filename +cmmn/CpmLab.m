classdef CpmLab < handle
% CPMLAB    Instance of experiment interface for usage in the cpm lab.
    properties
        matlabParticipant
    end

    properties (Access=private)
        vehicle_ids
        reader_vehicleStateList
        reader_systemTrigger
        writer_vehicleCommandTrajectory
        writer_vehicleCommandPathTracking
        writer_vehicleCommandDirect
        writer_readyStatus
        trigger_stop
        out_of_map_limits
        stop
        t_now_nanos
    end
    
    methods
        function obj = CpmLab(vehicle_ids)
            obj.vehicle_ids = vehicle_ids;
            assert(issorted(obj.vehicle_ids));
            
            obj.stop = true;
 
            obj.init_dds();
        end
        
        function send_ready_message(obj)

            % Sync start with infrastructure
            % Send ready signal for all assigned vehicle ids
            disp('Sending ready signal');
            for iVehicle = obj.vehicle_ids
                ready_msg = ReadyStatus;
                ready_msg.source_id = strcat('hlc_', num2str(iVehicle));
                ready_stamp = TimeStamp;
                ready_stamp.nanoseconds = uint64(0);
                ready_msg.next_start_stamp = ready_stamp;
                obj.writer_readyStatus.write(ready_msg);
            end

            % Wait for start or stop signal
            obj.wait_for_middleware();
        end
        
        function sample = measure(obj)
            [sample, ~, sample_count, ~] = obj.reader_vehicleStateList.take();
            if (sample_count > 1)
                warning('Received %d samples, expected 1. Correct middleware period? Missed deadline?', sample_count);
                warning('Using newest sample.');
                sample = sample(end);
            elseif(sample_count == 0)
                obj.stop = true;
                error('CpmLab:measure',"No message received from middleware. Stopping experiment.")
            end
            obj.t_now_nanos = uint64(sample.t_now);

            obj.out_of_map_limits = false(numel(obj.vehicle_ids),1);
        end
        
        function apply_path_tracking(obj, speed, path_points, dt_valid_after_nanos)
            nVeh = numel(obj.vehicle_ids);
            for iVeh = 1:nVeh
                apply_path_tracking_single_veh(obj, speed(iVeh,1), path_points, dt_valid_after_nanos, obj.vehicle_ids(iVeh));
            end
        end
        
        function apply_path_tracking_single_veh(obj, speed, path_points, dt_valid_after_nanos, veh_id)
            vehicle_command_path_tracking = VehicleCommandPathTracking;
            vehicle_command_path_tracking.vehicle_id = uint8(veh_id);
            vehicle_command_path_tracking.path = path_points;
            vehicle_command_path_tracking.speed = speed;
            vehicle_command_path_tracking.header.create_stamp.nanoseconds = ...
                uint64(obj.t_now_nanos);
            vehicle_command_path_tracking.header.valid_after_stamp.nanoseconds = ...
                uint64(obj.t_now_nanos + dt_valid_after_nanos);
            obj.writer_vehicleCommandPathTracking.write(vehicle_command_path_tracking);
        end

        function result = is_stop(obj)
            obj.read_system_trigger();
            if any(obj.out_of_map_limits)
                obj.stop = true;
            end
            result = obj.stop;
        end
        
        function end_run(~)
            disp('End')   
        end
        
    end
    
    methods (Access=private)    
        % helper function
        function stop_experiment = is_veh_at_map_border(~, trajectory_points)
            % Vehicle command timeout is 1000 ms after the last valid_after_stamp,
            % so vehicle initiates stop between third and fourth trajectory point
            vhlength = 0.25;
            x_min =  vhlength + 0;
            x_max = -vhlength + 4.5;
            y_min =  vhlength + 0;
            y_max = -vhlength + 4.0;
            px = trajectory_points(4).px;
            py = trajectory_points(4).py;
            stop_experiment = x_min>px || px>x_max ...
                           || y_min>py || py>y_max;
        end

        % This function is intended to be used by every single Matlab HLC script; it loads all required files and sets up all required writers/reader
        % WARNING: The state reader waitset does not get initialized here, in case you want to do something else
        % CAVE `matlabParticipant`must be stored for RTI DDS somewhere in the workspace  (so it doesn't get gc'ed)
        function init_dds(obj)
            script_directoy = fileparts([mfilename('fullpath') '.m']);
            software_directory = fullfile(script_directoy,'../../..');

            % Import IDL files from cpm library
            dds_idl_matlab = fullfile(software_directory,'cpm_lib/dds_idl_matlab/');
            assert(isfolder(dds_idl_matlab),...
                'Missing directory "%s".', dds_idl_matlab);
            assert(~isempty(dir([dds_idl_matlab, '*.m'])),...
                'No MATLAB IDL-files found in %s', dds_idl_matlab);
            addpath(dds_idl_matlab)

            % XML files for quality of service settings
            middleware_local_qos_xml = fullfile(software_directory, 'middleware/build/QOS_LOCAL_COMMUNICATION.xml');
            assert(isfile(middleware_local_qos_xml),...
                'Missing middleware local QOS XML "%s"', middleware_local_qos_xml);
            
            ready_trigger_qos_xml = fullfile(software_directory,'high_level_controller/examples/matlab/QOS_READY_TRIGGER.xml');
            assert(isfile(ready_trigger_qos_xml),...
                'Missing ready trigger QOS XML "%s"', ready_trigger_qos_xml);
            
            setenv("NDDS_QOS_PROFILES", ['file://' ready_trigger_qos_xml ';file://' middleware_local_qos_xml]);
            
            %% variables for the communication
            matlabStateTopicName = 'vehicleStateList';
            matlabCommandTrajectoryTopicName = 'vehicleCommandTrajectory';
            matlabCommandPathTrackingTopicName = 'vehicleCommandPathTracking';
            matlabCommandDirectTopicName = 'vehicleCommandDirect';
            systemTriggerTopicName = 'systemTrigger';
            readyStatusTopicName = 'readyStatus';
            obj.trigger_stop = uint64(18446744073709551615);

            %% create participants
            matlab_domain_id = 1;
            obj.matlabParticipant = DDS.DomainParticipant('MatlabLibrary::LocalCommunicationProfile', matlab_domain_id);

            %% create reader and writer
            obj.reader_vehicleStateList = DDS.DataReader(DDS.Subscriber(obj.matlabParticipant), 'VehicleStateList', matlabStateTopicName);
            obj.writer_vehicleCommandTrajectory = DDS.DataWriter(DDS.Publisher(obj.matlabParticipant), 'VehicleCommandTrajectory', matlabCommandTrajectoryTopicName);
            obj.writer_vehicleCommandPathTracking = DDS.DataWriter(DDS.Publisher(obj.matlabParticipant), 'VehicleCommandPathTracking', matlabCommandPathTrackingTopicName);
            obj.writer_vehicleCommandDirect = DDS.DataWriter(DDS.Publisher(obj.matlabParticipant), 'VehicleCommandDirect', matlabCommandDirectTopicName);
            obj.reader_systemTrigger = DDS.DataReader(DDS.Subscriber(obj.matlabParticipant), 'SystemTrigger', systemTriggerTopicName, 'TriggerLibrary::ReadyTrigger');
            obj.writer_readyStatus = DDS.DataWriter(DDS.Publisher(obj.matlabParticipant), 'ReadyStatus', readyStatusTopicName, 'TriggerLibrary::ReadyTrigger');

            % Set reader properties
            obj.reader_vehicleStateList.WaitSet = true;
            obj.reader_vehicleStateList.WaitSetTimeout = 2; % [s]
        end

        function wait_for_middleware(obj)
            disp('Waiting 120 seconds for start or stop signal');
            obj.reader_systemTrigger.WaitSet = true;
            obj.reader_systemTrigger.WaitSetTimeout = 120; % [s]
            obj.read_system_trigger();
            obj.reader_systemTrigger.WaitSet = false;
        end


        function read_system_trigger(obj)
            %READ_SYSTEM_TRIGGER Reads system trigger topic for start and stop signals
            [trigger, ~, sample_count, ~] = obj.reader_systemTrigger.take();
            if sample_count > 0
                % Stop signal
                if trigger(end).next_start().nanoseconds() == obj.trigger_stop
                    obj.stop = true;
                % Start signal
                else
                    obj.stop = false;
                end
            end
        end
    end
end

