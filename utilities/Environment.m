classdef Environment < Simulator
    properties
        logger
        N
        CoM_height_target
        episode_length
        dt_control
        vx_cmd
        step_period
        progress_within_step
        phase
        dphase
        swing_height
        z_init
        disable_logging
        terminated
        obs
    end

    methods
        function obj = Environment(logger, z_init, params, N, dt_mpc, dt_sim, q_idx, q_dot_idx, decimation, ...
                                   rest_coeff, fric_coeff, ground_height, restitution_coeff_joint, ...
                                   q_max_val, q_min_val, joint_idx, headless, disable_logging, ...
                                   CoM_fn, CoM_dot_fn, A_fn, b_fn, ...
                                   pos_front_ee, vel_front_ee, J_front_ee, ...
                                   pos_hind_ee, vel_hind_ee, J_hind_ee, keypoints_fn)
            % Initialize the Environment by calling the parent constructor
            obj@Simulator(params, dt_sim, q_idx, q_dot_idx, decimation, ...
                          rest_coeff, fric_coeff, ground_height, restitution_coeff_joint, ...
                          q_max_val, q_min_val, joint_idx, headless, ...
                          CoM_fn, CoM_dot_fn, A_fn, b_fn, ...
                          pos_front_ee, vel_front_ee, J_front_ee, ...
                          pos_hind_ee, vel_hind_ee, J_hind_ee, keypoints_fn);

            obj.logger = logger;
            obj.N = N;
            obj.CoM_height_target = 0.14;
            obj.episode_length = 0;
            obj.dt_control = dt_sim * decimation;
            obj.vx_cmd = 0;
            obj.step_period = 24; % Corresponds to 0.2s
            obj.progress_within_step = 0;
            obj.phase = 0;
            obj.dphase = 1 / ((obj.step_period-1) * obj.dt_control);
            obj.swing_height = 0.01;
            obj.z_init = z_init;
            obj.disable_logging = disable_logging;
            obj.terminated = false;

            % Initialize the observation structure
            obj.obs = struct();
            obj.obs.t = 0;
            obj.obs.z = z_init;
            obj.obs.z_ref = repmat([0; obj.CoM_height_target; 0; pi/4; -pi/2; pi/4; -pi/2;
                                    0; 0; 0; 0; 0; 0; 0;], 1, obj.N+1);
            obj.obs.X_ref_val = zeros(6, obj.N+1);
            obj.obs.x0_val = zeros(6,1);
            obj.obs.dt_mpc = dt_mpc;
            obj.obs.foot_contact_cmd = true; % true means front foot contact
            obj.obs.contact_boolean_val = zeros(2,obj.N);
            obj.obs.foot_ref_val = zeros(2,obj.N+1);
            obj.obs.p_front_val = zeros(2,obj.N);
            obj.obs.p_hind_val = zeros(2,obj.N);
            obj.obs.foot_ref_position = zeros(2,2); % Front foot (x,y), Hind foot (x,y)
            obj.obs.foot_ref_velocity = zeros(2,2); % Front foot (x,y), Hind foot (x,y)
            obj.obs.foot_ref_acceleration = zeros(2,2); % Front foot (x,y), Hind foot (x,y)
            obj.obs.prev_step_commands = [0.1 0; -0.1 0]; % Front foot (x,y), Hind foot (x,y)
            obj.obs.step_commands = [0.1 0; -0.1 0]; % Front foot (x,y), Hind foot (x,y)

        end

        function reset(obj)
            % Reset the environment
            obj.episode_length = 0;
            obj.vx_cmd = 0;
            obj.progress_within_step = 0;
            obj.phase = 0;

            obj.obs.t = 0;
            obj.obs.z = obj.z_init;
            obj.obs.X_ref_val = zeros(6, obj.N+1);
            obj.obs.x0_val = zeros(6,1);
            obj.obs.foot_contact_cmd = true; % true means front foot contact
            obj.obs.foot_ref_val = zeros(2,obj.N+1);
            obj.obs.p_front_val = zeros(2,obj.N);
            obj.obs.p_hind_val = zeros(2,obj.N);
            obj.obs.foot_ref_position = zeros(2,2); % Front foot (x,y), Hind foot (x,y)
            obj.obs.foot_ref_velocity = zeros(2,2); % Front foot (x,y), Hind foot (x,y)
            obj.obs.foot_ref_acceleration = zeros(2,2); % Front foot (x,y), Hind foot (x,y)
            obj.obs.prev_step_commands = [0.1 0; -0.1 0]; % Front foot (x,y), Hind foot (x,y)
            obj.obs.step_commands = [0.1 0; -0.1 0]; % Front foot (x,y), Hind foot (x,y)
        end

        function check_termination(obj)
            % Check if the termination condition is met
            obj.terminated = obj.obs.z(2) < obj.ground_height;
            obj.terminated = obj.terminated | obj.obs.z(2) > 0.3;
            obj.terminated = obj.terminated | obj.obs.z(3) < -pi/2;
            obj.terminated = obj.terminated | obj.obs.z(3) > pi/2;

            % If termination condition met, reset the environment
            if obj.terminated
                obj.reset();
                obj.terminated = false; % Reset the flag
            end
        end

        function obs = step(obj, u)
            
            % Run the simulation for the given decimation and render and update the observation
            for i = 1:obj.decimation
                % Run the simulation for the given decimation
                obj.obs.z = obj.simulate(obj.obs.z, u);
            end

            % Render the simulation
            if ~obj.headless
            obj.render(obj.obs, obj.vx_cmd);
            end

            % Post physics step
            obj.post_physics_step();

            % Update the episode_length and time stamp
            obj.episode_length = obj.episode_length + 1;
            obj.obs.t = obj.episode_length * obj.dt_control;

            % Check termination condition
            obj.check_termination();

            % Log data
            if ~obj.disable_logging
            frame = getframe(gcf);
            keypoints = full(obj.keypoints_fn(obj.obs.z, obj.p));
            obj.logger.log(frame, obj.obs, u, obj.vx_cmd, keypoints);
            end
            % Return the updated observation
            obs = obj.obs;
        end

        function post_physics_step(obj)
            % Update the observation after physics calculations
            obj.swing_height = 0.01 + abs(obj.vx_cmd)*0.10;
            obj.swing_height = min(obj.swing_height, 0.04);
            obj.progress_within_step = mod(obj.episode_length, obj.step_period);
            obj.phase = obj.progress_within_step / (obj.step_period-1);
            obj.obs.foot_contact_cmd = mod(floor(obj.episode_length / obj.step_period), 2) == 0;
            obj.update_obs();
            obj.update_foot_ref();
        end

        function update_obs(obj)
            % Update generalized velocity for the reference trajectory
            obj.obs.z_ref(8,:) = obj.vx_cmd;

            % Update generalized position
            obj.obs.z_ref(1,1) = obj.obs.z(1);

            for k = 2:obj.N+1
                obj.obs.z_ref(1,k) = obj.obs.z_ref(1,k-1) + obj.vx_cmd * obj.obs.dt_mpc;
            end

            % Update x0
            p_CoM = full(obj.CoM_fn(obj.obs.z, obj.p));
            v_CoM = full(obj.CoM_dot_fn(obj.obs.z, obj.p));
            obj.obs.x0_val = [obj.obs.z(3); p_CoM(1); p_CoM(2);
                              obj.obs.z(10); v_CoM(1); v_CoM(2)];

            % Update X_ref [theta x y w vx vy]
            obj.obs.X_ref_val(1,:) = obj.z_init(3);
            obj.obs.X_ref_val(2,1) = p_CoM(1);
            obj.obs.X_ref_val(3,:) = obj.CoM_height_target;
            obj.obs.X_ref_val(5,:) = obj.vx_cmd;

            for k = 2:obj.N+1
                obj.obs.X_ref_val(2,k) = obj.obs.X_ref_val(2,k-1) + obj.vx_cmd * obj.obs.dt_mpc;
            end

            % Update contact sequence
            switch_idx = min(obj.N, obj.step_period - obj.progress_within_step);
            time_steps = 0:(obj.N-1);
            mask = time_steps < switch_idx;
            if obj.obs.foot_contact_cmd % Front foot is in contact
                obj.obs.contact_boolean_val(1, :) = mask;
                obj.obs.contact_boolean_val(2, :) = ~mask;
            else % Hind foot is in contact
                obj.obs.contact_boolean_val(1, :) = ~mask;
                obj.obs.contact_boolean_val(2, :) = mask;
            end
            if obj.episode_length < obj.step_period
                obj.obs.contact_boolean_val(:,1:switch_idx) = 1;
            end
            % obj.obs.contact_boolean_val(:) = 1;
            % * Update foot reference
            % max_height = 0.02;
            % predicted_progress_within_step = mod(obj.episode_length + (0:obj.N), obj.step_period);
            % predicted_phase = predicted_progress_within_step / (obj.step_period - 1);
            % obj.obs.foot_ref_val = obj.obs.contact_boolean_val .* (4 * max_height * predicted_phase .* (1 - predicted_phase));
        end

        function update_foot_ref(obj)
            % * Update foot reference using Raibert Heuristic
            keypoints = full(obj.keypoints_fn(obj.obs.z, obj.p));
            q = obj.obs.z(obj.q_idx);
            q_dot = obj.obs.z(obj.q_dot_idx);
            base_position = q(1:2);
            base_velocity = q_dot(1:2);
            p_CoM = full(obj.CoM_fn(obj.obs.z, obj.p));
            v_CoM = full(obj.CoM_dot_fn(obj.obs.z, obj.p));
            front_foot_position = full(obj.pos_front_ee(obj.obs.z, obj.p));
            hind_foot_position = full(obj.pos_hind_ee(obj.obs.z, obj.p));
            front_hip_position = keypoints(:,5);
            hind_hip_position = keypoints(:,10);
            remaining_period = obj.step_period - obj.progress_within_step;
            % k = sqrt(p_CoM(2) / obj.p(end));
            % % k = sqrt(base_position(2) / obj.p(end));
            % % k = 0;
            % p_symmetry = 0.5 * remaining_period * obj.dt_control * v_CoM(1) + ...
            %              1.0 * k * (v_CoM(1) - obj.vx_cmd);
            k = sqrt(base_position(2) / obj.p(22));
            p_symmetry = 0.5 * remaining_period * obj.dt_control * base_velocity(1) + ...
                         k * (base_velocity(1) - obj.vx_cmd);
            % p_symmetry = 0.5 * remaining_period * obj.dt_control * base_velocity(1) + ...
            %              k * (base_velocity(1) - obj.vx_cmd);

            if obj.obs.foot_contact_cmd % Front foot in contact
                obj.obs.step_commands(1,1) = front_foot_position(1);
                obj.obs.prev_step_commands(1,1) = front_foot_position(1);
                obj.obs.step_commands(2,1) = hind_hip_position(1) + p_symmetry;
            else % Hind foot in contact
                obj.obs.step_commands(2,1) = hind_foot_position(1);
                obj.obs.prev_step_commands(2,1) = hind_foot_position(1);
                obj.obs.step_commands(1,1) = front_hip_position(1) + p_symmetry;
            end

            obj.obs.p_front_val = repmat(obj.obs.step_commands(1,:)', 1, obj.N);
            obj.obs.p_hind_val = repmat(obj.obs.step_commands(2,:)', 1, obj.N);

            P0 = obj.obs.prev_step_commands;
            P0(:,2) = 0;
            P3 = obj.obs.step_commands;
            P3(:,2) = 0;
            P1 = P0 + (P3 - P0)/3;
            P1(:,2) = obj.swing_height;
            P2 = P0 + 2 * (P3 - P0)/3;
            P2(:,2) = obj.swing_height;
            t = obj.phase;
            
            obj.obs.foot_ref_position = (1-t)^3 * P0 + 3 * (1-t)^2 * t * P1 + 3 * (1-t) * t^2 * P2 + t^3 * P3;
            obj.obs.foot_ref_velocity = (3 * (1-t)^2 * (P1 - P0) + 6 * (1-t) * t * (P2 - P1) + 3 * t^2 * (P3 - P2)) * obj.dphase;
            obj.obs.foot_ref_acceleration = (6 * (1-t) * (P2 - 2 * P1 + P0) + 6 * t * (P3 - 2 * P2 + P1)) * obj.dphase^2;
            
            if obj.obs.foot_contact_cmd
                obj.obs.foot_ref_position(1,2) = 0;
                obj.obs.foot_ref_velocity(1,2) = 0;
                obj.obs.foot_ref_acceleration(1,2) = 0;
            else
                obj.obs.foot_ref_position(2,2) = 0;
                obj.obs.foot_ref_velocity(2,2) = 0;
                obj.obs.foot_ref_acceleration(2,2) = 0;
            end

        end
        
    end
end
