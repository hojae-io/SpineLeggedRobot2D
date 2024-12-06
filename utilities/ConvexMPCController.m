% * Convex MPC Controller class
% * This class is used to create a Model Predictive Controller (MPC) object
% * and solve the optimization problem to generate control inputs.

classdef ConvexMPCController
    properties
        % * MPC parameters
        N
        dt
        p
        q_idx
        q_dot_idx
        total_mass
        gravity_acc
        total_weight
        I
        mu
        Kp
        Kd

        % * Function handles
        A_fn
        b_fn
        G_fn
        C_fn
        InvDynamics_fn
        pos_front_ee
        vel_front_ee
        J_front_ee
        dJ_front_ee
        pos_hind_ee
        vel_hind_ee
        J_hind_ee
        dJ_hind_ee

        % * Opti stack variables
        opti
        X
        U
        F_front
        F_hind
        X_ref
        x0
        p_front
        p_hind
        contact_boolean

        % * Opti functions
        opti_f_fn
    end
    
    methods
        function obj = ConvexMPCController(N, dt, params, q_idx, q_dot_idx, ...
                                           A_fn, b_fn, G_fn, C_fn, InvDynamics_fn, pos_front_ee, vel_front_ee, J_front_ee, dJ_front_ee, ...
                                           pos_hind_ee, vel_hind_ee, J_hind_ee, dJ_hind_ee)

            % * Constructor to initialize the CasADi optimization problem
            obj.N = N;
            obj.dt = dt;
            obj.p = params;
            obj.q_idx = q_idx;
            obj.q_dot_idx = q_dot_idx;
            obj.total_mass = params(1) + 2*sum(params(2:5)); % .39886;
            obj.gravity_acc = params(22);
            obj.total_weight = obj.total_mass * obj.gravity_acc;
            obj.I = params(6) + 2*sum(params(7:10)); % 0.00304004;
            obj.mu = 0.6;
            obj.Kp = diag([4500 4500]);
            obj.Kd = diag([58 58]);

            obj.A_fn = A_fn;
            obj.b_fn = b_fn;
            obj.G_fn = G_fn;
            obj.C_fn = C_fn;
            obj.InvDynamics_fn = InvDynamics_fn;
            obj.pos_front_ee = pos_front_ee;
            obj.vel_front_ee = vel_front_ee;
            obj.J_front_ee = J_front_ee;
            obj.dJ_front_ee = dJ_front_ee;
            obj.pos_hind_ee = pos_hind_ee;
            obj.vel_hind_ee = vel_hind_ee;
            obj.J_hind_ee = J_hind_ee;
            obj.dJ_hind_ee = dJ_hind_ee;

            % * Initialize the CasADi Opti stack
            obj.opti = casadi.Opti('conic');
            % obj.opti = casadi.Opti();

            % * Define the state and control variables
            obj.X = obj.opti.variable(6, obj.N+1); % [theta x y w vx vy]
            obj.U = obj.opti.variable(4, obj.N); % 4 control inputs
            obj.F_front = obj.U(1:2,:);
            obj.F_hind = obj.U(3:4, :);

            % * Define parameters
            obj.X_ref = obj.opti.parameter(6, obj.N+1);
            obj.x0 = obj.opti.parameter(6, 1);
            obj.p_front = obj.opti.parameter(2, obj.N);
            obj.p_hind = obj.opti.parameter(2, obj.N);
            obj.contact_boolean = obj.opti.parameter(2, obj.N);

            % * Define constant values
            Q = diag([200 200 2200 20 300 100]); % [theta x y w vx vy]
            R = 1e-6 * diag(ones(4,1));
            F_max = 30;
            F_min = 0;

            function A_mat = A_c()
                A_mat = [zeros(3,3), eye(3);
                         zeros(3,3), zeros(3,3)];
            end
            function B_mat = B_c(mass, p_f_fromCoM, p_h_fromCoM)
                inv_I = 1 / obj.I;
            
                B_mat = [zeros(3,2), zeros(3,2);
                         -inv_I * p_f_fromCoM(2), inv_I * p_f_fromCoM(1), -inv_I * p_h_fromCoM(2), inv_I * p_h_fromCoM(1);
                         eye(2)/mass, eye(2)/mass];
            end
            function g_vector = g_vec()
                g_vector = [zeros(3,1); 0; 0; -obj.gravity_acc];
            end

            % * Design cost function
            cost = 0;
            for k = 2:obj.N+1
                error = obj.X(:,k) - obj.X_ref(:,k);
                cost = cost + error' * Q * error;
            end
            for k = 1:obj.N
                cost = cost + obj.U(:,k)' * R * obj.U(:,k);
            end
            
            obj.opti.minimize(cost);

            % * Set initial condition constraints
            obj.opti.subject_to(obj.X(:, 1) == obj.x0);
            
            % * Set dynamic constraints
            for k = 1:obj.N
                % * Discrete-time dynamics constraints
                p_k = obj.X_ref(2:3, k);
                p_f_fromCoM = obj.p_front(:, k) - p_k;
                p_h_fromCoM = obj.p_hind(:, k) - p_k;
                dx = A_c() * obj.X(:, k) + B_c(obj.total_mass, p_f_fromCoM, p_h_fromCoM) * obj.U(:, k) + g_vec();
                obj.opti.subject_to(obj.X(:, k+1) - (obj.X(:, k) + obj.dt * dx) == 0);
            end

            % * Set input contraints
            for k = 1:obj.N
                % Contact constraints
                obj.opti.subject_to( obj.F_front(1, k) - obj.contact_boolean(1,k) * obj.mu * obj.F_front(2, k) <= 0);
                obj.opti.subject_to(-obj.F_front(1, k) - obj.contact_boolean(1,k) * obj.mu * obj.F_front(2, k) <= 0);
                obj.opti.subject_to( obj.F_front(2, k) - obj.contact_boolean(1,k) * F_max <= 0);
                obj.opti.subject_to(-obj.F_front(2, k) + obj.contact_boolean(1,k) * F_min <= 0);

                obj.opti.subject_to( obj.F_hind(1, k) - obj.contact_boolean(2,k) * obj.mu * obj.F_hind(2, k) <= 0);
                obj.opti.subject_to(-obj.F_hind(1, k) - obj.contact_boolean(2,k) * obj.mu * obj.F_hind(2, k) <= 0);
                obj.opti.subject_to( obj.F_hind(2, k) - obj.contact_boolean(2,k) * F_max <= 0);
                obj.opti.subject_to(-obj.F_hind(2, k) + obj.contact_boolean(2,k) * F_min <= 0);
            end

            % Solver settings and solve
            obj.opti.solver('proxqp');

            % % * Solver settings and solve
            % p_opts = struct('expand', false, ...
            %                 'print_time', false);
            % 
            % s_opts = struct('max_iter', 1000, ...
            %                 'linear_solver', 'ma27', ... % 'ma27', 'mumps'
            %                 'warm_start_init_point', 'yes', ...
            %                 'warm_start_bound_push', 1e-8, ...
            %                 'warm_start_mult_bound_push', 1e-8, ...
            %                 'mu_init', 1e-5, ...
            %                 'bound_relax_factor', 1e-9, ...
            %                 'print_level', 3);
            % 
            % % Set the solver with options
            % obj.opti.solver('ipopt', p_opts, s_opts);
 
        end
        
        function u = compute(obj, obs)
            % * Retrieve the observation
            z = obs.z;

            % * Solve the optimization problem and return the control input
            obj.opti.set_value(obj.X_ref, obs.X_ref_val);
            obj.opti.set_value(obj.x0, obs.x0_val);
            obj.opti.set_value(obj.p_front, obs.p_front_val);
            obj.opti.set_value(obj.p_hind, obs.p_hind_val);
            obj.opti.set_value(obj.contact_boolean, obs.contact_boolean_val);

            soln = obj.opti.solve();
            X_soln = soln.value(obj.X);
            U_soln = soln.value(obj.U);
            F_front_soln = soln.value(obj.F_front);
            F_hind_soln = soln.value(obj.F_hind);
            
            J_f = obj.J_front_ee(z, obj.p);
            J_h = obj.J_hind_ee(z, obj.p);
            dJ_f = obj.dJ_front_ee(z, obj.p);
            dJ_h = obj.dJ_hind_ee(z, obj.p);
            M = obj.A_fn(z, obj.p);
            C = obj.C_fn(z, obj.p);
            G = obj.G_fn(z, obj.p);

            if obs.foot_contact_cmd % Front foot contact
                u = - J_f' * F_front_soln(:,1);
                p_hind_error = obs.foot_ref_position(2,:)' - full(obj.pos_hind_ee(z,obj.p));
                v_hind_error = obs.foot_ref_velocity(2,:)' - full(obj.vel_hind_ee(z,obj.p));
                Lambda_h = eye(size(J_h,1)) / (J_h / M * J_h');
                mu_h = Lambda_h * J_h / M * C - Lambda_h * dJ_h * z(obj.q_dot_idx);
                rho_h = Lambda_h * J_h / M * G;
                u = u + J_h' * (Lambda_h * (obs.foot_ref_acceleration(2,:)' + obj.Kp * p_hind_error + obj.Kd * v_hind_error) + mu_h + rho_h);
            else % Hind foot contact
                u = - J_h' * F_hind_soln(:,1);
                p_front_error = obs.foot_ref_position(1,:)' - full(obj.pos_front_ee(z,obj.p));
                v_front_error = obs.foot_ref_velocity(1,:)' - full(obj.vel_front_ee(z,obj.p));
                Lambda_f = eye(size(J_f,1)) / (J_f / M * J_f');
                mu_f = Lambda_f * J_f / M * C - Lambda_f * dJ_f * z(obj.q_dot_idx);
                rho_f = Lambda_f * J_f / M * G;
                u = u + J_f' * (Lambda_f * (obs.foot_ref_acceleration(1,:)' + obj.Kp * p_front_error + obj.Kd * v_front_error) + mu_f + rho_f); % Joint space impedance control
            end
            if sum(obs.contact_boolean_val(:,1)) == 2
                u = - J_f' * F_front_soln(:,1) - J_h' * F_hind_soln(:,1);
            end
            u = u(4:7);

            % * Update initial guess
            obj.opti.set_initial(obj.X, X_soln);
            obj.opti.set_initial(obj.U, U_soln);
        end
        
    end
end

%% Explanation
%{
1. u_tau = {Front hip, Front knee, Hind hip, Hind knee}
2. u_GRF = {Front foot Fx, Front foot Fy, Hind foot Fx, Hind foot Fy}
mpc_controller.opti.debug.value(mpc_controller.q)
mpc_controller.opti.debug.show_infeasibilities(1e-3)
%}
