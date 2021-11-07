classdef KalmanFilter < handle
%KALMANFILTER  Create a kalman filter.
%   kalmf = KALMANFILTER(MODEL,Q,R,X_INIT)
%   creates the kalman filter object where
%   MODEL must be a delay free, discretized SS model with fields A, B and C.
%   Q is the weighting matrix describing the process noise,
%   dimensions (nx,nx)
%   R is the weighting matrix describing the measurement noise,
%   dimensions (ny,ny)
%   X_INIT is the initial value of the state vector, dimensions (nx,1).
%   The state vector can also be set during operation by changing the
%   member X_K_MINUS_ONE.

    properties
        x_k_minus_one
        P_k_minus_one
        model
        Q
        R
    end

    methods
        function obj = KalmanFilter(model,Q,R,x_init)
            obj.model = model;
            obj.Q = Q;
            obj.R = R;
            
            if (nargin==4)
                obj.x_k_minus_one = x_init;
            else
                obj.x_k_minus_one = zeros(size(model.A,1),1);
            end
            obj.P_k_minus_one = eye(size(model.A));
        end

        function [x_k,P_k] = step(obj,ym,u_k_minus_one)
            % https://de.wikipedia.org/wiki/Kalman-Filter
            % Prediction
            x_pred = obj.model.A*obj.x_k_minus_one + obj.model.B*u_k_minus_one;
            P_pred = obj.model.A*obj.P_k_minus_one*obj.model.A' + obj.Q;

            % Correction
            K_k = P_pred*obj.model.C'/(obj.model.C*P_pred*obj.model.C'+obj.R);
            x_k = x_pred + K_k*(ym-obj.model.C*x_pred);
            P_k = (eye(size(obj.model.A))-K_k*obj.model.C)*P_pred;
            
            % Remember
            obj.x_k_minus_one = x_k;
            obj.P_k_minus_one = P_k;

        end
    end
end