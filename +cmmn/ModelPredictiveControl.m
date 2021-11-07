classdef ModelPredictiveControl < cmmn.InterfaceController
%MODELPREDICTIVECONTROL  Create a model predictive controller
%   mpcObj = MODELPREDICTIVECONTROL(MODEL,HP,HU,UMIN,UMAX,DUMIN,DUMAX,YMIN,YMAX,Q,R,Q_KALMAN,R_KALMAN)
%   creates the MPC object where
%   MODEL must be a delay free, discretized SS model with fields A, B and C.
%   HP is the prediction horizon specified as a scalar number.
%   HU is the control horizon specified as a scalar number.
%   UMIN is the minimum value for the control input,
%   dimensions (nu,1).
%   UMAX is the maximum value for the control input,
%   dimensions (nu,1).
%   DUMIN is the minimum value for the control input change per time step,
%   specified as a scalar number.
%   DUMAX is the maximum value for the control input change per time step,
%   specified as a scalar number.
%   YMIN is the minimum value of the system output over the prediction horizon.
%   Needs to have dimension (ny*HP,1), where ny is the number of system outputs.
%   The vector is ny outputs stacked at each time k+1, k+2, ... k+HP.
%   E.g. [y_1(k+1),...,y_ny(k+1), y_1(k+2),...,y_ny(k+1), ..., y_1(k+HP),...,y_ny(k+HP)]'
%   Can also be set at each control step in function STEP.
%   YMAX is the maximum value of the system output over the prediction horizon.
%   Similar to YMIN.
%   Q is the weighting matrix for system outputs y, dimensions=(ny,ny)
%   R is the weighting matrix for control input changes du, dimensions=(nu,nu)
%   Q_KALMAN is the weighting matrix for for the integrated kalman filter,
%   describing the process noise, dimensions=(nx,nx)
%   R_KALMAN is the weighting matrix for for the integrated kalman filter,
%   describing the measurement noise, dimensions=(ny,ny)
%
%   Remember to set the initial state vector of the kalman filter to x_init with
%   mpcObj.observer.x_k_minus_one = x_init
%
%   MODELPREDICTIVECONTROL properties:
%       OBSERVER - observer of system states
%
%   MODELPREDICTIVECONTROL methods:
%       STEP - solves an optimization problem to generate control input u
%
%   See also KALMANFILTER.

    properties
        % observer - observer of system states
        % See also KALMANFILTER
        observer
        hp
        hu
        model
    end
    
    properties (Access=private)
        psi
        gamma
        theta
        psi_y
        gamma_y
        theta_y
        C_hp
        Q
        R
        nu
        ny
        umin
        umax
        dumin
        dumax
        ymin
        ymax
        dt
        u_k_minus_one
    end

    methods
        function obj = ModelPredictiveControl(model,hp,hu,umin,umax,dumin,dumax,ymin,ymax,q,r,Q_kalman,R_kalman)
            obj.model = model;
            obj.observer = cmmn.KalmanFilter(model,Q_kalman,R_kalman);

            obj.hp = hp;
            obj.hu = hu;
            obj.nu = size(obj.model.B,2);
            obj.ny = size(obj.model.C,1);
            obj.u_k_minus_one = zeros(obj.nu,1);
            obj.umin = umin;
            obj.umax = umax;
            obj.dumin = dumin;
            obj.dumax = dumax;
            obj.ymin = ymin;
            obj.ymax = ymax;

            % initialize mpc matrices
            psi_mat = model.A;
            for i=2:hp
                psi_mat = [psi_mat;model.A^i];
            end
    
            gamma_mat = model.B;
            for i=2:hp
                g_add = model.B;
                for j=1:(i-1)
                    g_add = g_add + model.A^j*model.B;
                end
                gamma_mat = [gamma_mat;g_add];
            end
    
            theta_mat = [];
            for i=1:hp
                theta_row = [];
                for j=1:hu
                    theta_next = zeros(size(model.B));
                    for k=0:(i-j)
                        theta_next = theta_next + model.A^k*model.B;
                    end
                    theta_row = [theta_row theta_next];
                end
                theta_mat = [theta_mat;theta_row];
            end

            % Matrix to get y
            % blkdiag needs comma separated output, so use cell
            Ccell = repmat({model.C},1,hp);
            obj.C_hp = blkdiag(Ccell{:});
    
            Q = q;
            for i=2:hp
                Q = blkdiag(Q, q);
            end
    
            R = r;
            for i=2:hu
                R = blkdiag(R, r);
            end

            obj.psi = psi_mat;
            obj.gamma = gamma_mat;
            obj.theta = theta_mat;
            obj.psi_y = obj.C_hp*psi_mat;
            obj.gamma_y = obj.C_hp*gamma_mat;
            obj.theta_y = obj.C_hp*theta_mat;
            obj.Q = Q;
            obj.R = R;
        end


        function setup(obj,x_init)
            obj.observer.x_k_minus_one = x_init;
        end


        function [u,y] = step(obj,ym,ref,ymin,ymax)
            %STEP solves an optimization problem to generate control input u
            % U: control input for the next timestep. dimensions=(nu,1)
            % Y: output prediction as a vector of stacked vectors of y(k) for
            %     timepoints k=1,...,Hp. dimensions=(ny*Hp,1)
            % [U,Y] = STEP(YM, REF)
            % YM: current measurement, dimensions=(ny,1)
            % REF: reference for y over the prediction horizon,
            %     dimensions=(ny*Hp,1). If dimensions=(ny,1), vector 
            %     will be repeated Hp times.
            % [U,Y] = STEP(YM, REF, YMIN, YMAX)
            % as above, specification of output constraints possible
            % YMIN: lower bound for y over the prediction horizon,
            %     dimensions=(ny*Hp,1). If dimensions=(ny,1), vector 
            %     will be repeated Hp times.
            % YMAX: upper bound for y over the prediction horizon,
            %     dimensions=(ny*Hp,1). If dimensions=(ny,1), vector 
            %     will be repeated Hp times.
            
            if (nargin == 5)
                obj.ymin = ymin;
                obj.ymax = ymax;
            end

            % Observe state
            [x_k,~] = obj.observer.step(ym,obj.u_k_minus_one);

            % objective
            if (size(ref,1)==obj.ny)
                ref = repmat(ref,obj.hp,1);
            end
            y_free = obj.psi_y*x_k + obj.gamma_y*obj.u_k_minus_one;
            assert(size(ref,2)==1);
            assert(size(ref,1)==obj.hp*obj.ny);
            f = 2 * obj.theta_y' * obj.Q * (y_free - ref);
            h = 2 * (obj.theta_y' * obj.Q * obj.theta_y + obj.R);
            
            % u constraints
            Aineq_umax = kron(tril(ones(obj.hu,obj.hu)),eye(obj.nu));
            Aineq_umin = -Aineq_umax;
            Aineq_u = [...
                Aineq_umax;...
                Aineq_umin...
            ];

            bineq_umax =   repmat(obj.umax, obj.hu, 1) - repmat(obj.u_k_minus_one, obj.hu, 1);
            bineq_umin = -(repmat(obj.umin, obj.hu, 1) - repmat(obj.u_k_minus_one, obj.hu, 1));
            bineq_u = [...
                bineq_umax;...
                bineq_umin...
            ];
        
            lb = obj.dumin;
            if (numel(lb)==1)
                lb = repmat(lb,obj.nu,1);
            end
            if (numel(lb)==obj.nu)
                lb = repmat(lb,obj.hu,1);
            end
            ub = obj.dumax;
            if (numel(ub)==1)
                ub = repmat(ub,obj.nu,1);
            end
            if (numel(ub)==obj.nu)
                ub = repmat(ub,obj.hu,1);
            end
                

            % y constraints
            Aineq_ymax =  obj.theta_y;
            Aineq_ymin = -obj.theta_y;
            
            Aineq_y = [...
                Aineq_ymax;...
                Aineq_ymin...
            ];
            
            % extend lower bounds of system output to prediction horizon
            if (size(obj.ymin,1) == obj.ny)
                obj.ymin = repmat(obj.ymin,obj.hp,1);
            end
            % extend upper bounds of system output to prediction horizon
            if (size(obj.ymax,1) == obj.ny)
                obj.ymax = repmat(obj.ymax,obj.hp,1);
            end
            y_constraint = [obj.ymax; -obj.ymin];
            bineq_y = y_constraint - [y_free;-y_free];
            
            Aineq = [...
                Aineq_u;...
                Aineq_y...
            ];
            bineq = [...
                bineq_u;...
                bineq_y...
            ];


            % solve program
            options = optimset('Display', 'off', 'LargeScale', 'off');
            Delta_u_sol = quadprog(h,f,Aineq,bineq,[],[],lb,ub,[],options);
            % Delta_u_sol is
            % [u_1(k),
            %  u_2(k),
            %  ...,   
            %  u_nu(k),
            %  u_1(k+1),
            %  ...,
            %  u_nu(k+1),
            %  ...,
            %  u_nu(k+hu)]'
            if (isempty(Delta_u_sol))
                warning("No feasible solution found, setting v=0.");
                Delta_u_sol = zeros(obj.nu*obj.hu,1);
                Delta_u_sol(1:obj.nu) = -obj.u_k_minus_one;
            end

            d_u_k = Delta_u_sol(1:obj.nu);
            % compute future states and outputs
            u = obj.u_k_minus_one + d_u_k;
            y = obj.psi_y*x_k + obj.gamma_y*obj.u_k_minus_one + obj.theta_y*Delta_u_sol;
            
            % save last control input
            obj.u_k_minus_one = u;
        end
    end
end