classdef HlcIdentification < cmmn.InterfaceHlc
% HlcIdentification implements an HLC for model identification
    properties
        mt
        Ts
        path_points
    end

    methods
        function obj = HlcIdentification(Ts,vehicle_ids)
            obj = obj@cmmn.InterfaceHlc(vehicle_ids);

            obj.Ts = Ts;

            nVeh = numel(vehicle_ids);
            assert(nVeh==1);

            obj.path_points = cmmn.outer_lane_path();
            obj.mt = cmmn.MeasurementTransformer(obj.path_points,vehicle_ids);
        end
        
        function on_first_timestep(obj, vehicle_state_list)
            on_first_timestep@cmmn.InterfaceHlc(obj,vehicle_state_list);
        end

        function on_each_timestep(obj, vehicle_state_list)
            on_each_timestep@cmmn.InterfaceHlc(obj,vehicle_state_list);

            % pose to distance on path f/e veh
            y = obj.mt.measure_longitudinal(vehicle_state_list);
            disp(y)

            % Compute control action
            % ----------------------
            u = 1;

            % Apply control action
            % --------------------
            obj.cpmLab.apply_path_tracking(u, obj.path_points, obj.dt_valid_after_nanos)
        end

        function on_stop(obj)
            on_stop@cmmn.InterfaceHlc(obj);
            % TODO plot results, see plot_platooning.m
        end

    end
end
