classdef Hlc < cmmn.InterfaceHlc
% Hlc implements an HLC for position control of a single vehicle
    properties
        mpc
        mt
        yref
        Ts
        path_points
    end

    methods
        function obj = Hlc(Ts,vehicle_ids)
            obj = obj@cmmn.InterfaceHlc(vehicle_ids);

            obj.Ts = Ts;

            nVeh = numel(vehicle_ids);
            assert(nVeh==1);

            obj.path_points = cmmn.outer_lane_path();
            obj.mt = cmmn.MeasurementTransformer(obj.path_points,vehicle_ids);
            
            %% controller
            % MPC Params
            Hp            = 0;
            Hu            = 0;
            a_min         = 0;
            a_max         = 0;
            v_min         = 0;
            v_max         = 0;

            vehicle_model = cmmn.longitudinal_model(obj.Ts);
            % MPC Init
            q             = 0;
            r             = 0;

            Q_kalman      = 0;
            R_kalman      = 0;
            ymin          = 0;
            ymax          = 0;

            obj.mpc       = cmmn.ModelPredictiveControl();
        end
        
        function on_first_timestep(obj, vehicle_state_list)
            on_first_timestep@cmmn.InterfaceHlc(obj,vehicle_state_list);

            x = zeros(size(obj.mpc.model.A,1),1);
            y = obj.mt.measure_longitudinal(vehicle_state_list);
            x(1:2) = y;  % first two states are measured
            obj.mpc.setup(x);

            % Reference generation
            obj.yref = @(t) 0;
        end

        function on_each_timestep(obj, vehicle_state_list)
            on_each_timestep@cmmn.InterfaceHlc(obj,vehicle_state_list);

            % pose to distance on path f/e veh
            y = obj.mt.measure_longitudinal(vehicle_state_list);

            % Compute control action
            % ----------------------
            [u, ~] = obj.mpc.step(y,obj.yref(obj.t_exp));

            % Apply control action
            % --------------------
            obj.cpmLab.apply_path_tracking(u, obj.path_points, obj.dt_valid_after_nanos)
        end

        function on_stop(obj)
            on_stop@cmmn.InterfaceHlc(obj);
        end

    end
end
