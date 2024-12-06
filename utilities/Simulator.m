% * Simulator class
% * This class is used to simulate the system dynamics (robot dynamics + contact dynamics).

classdef Simulator < handle
    properties
        p
        dt_sim
        q_idx
        q_dot_idx
        decimation
        rest_coeff
        fric_coeff
        ground_height
        rest_coeff_joint
        q_max_val
        q_min_val
        joint_idx
        CoM_fn
        CoM_dot_fn
        A_fn
        b_fn
        pos_front_ee
        vel_front_ee
        J_front_ee
        pos_hind_ee
        vel_hind_ee
        J_hind_ee
        keypoints_fn

        % * Analysis properties
        F_fx
        F_fy
        F_hx
        F_hy

        % * Rendering properties
        headless
        fig
        h_title
        h_OOf
        h_OfBf
        h_AfCf
        h_BfDf
        h_CfEf
        h_OOh
        h_OhBh
        h_AhCh
        h_BhDh
        h_ChEh
        ground
        h_velocity_cmd_arrow
        force_scale
        h_Ff_arrow
        h_Fh_arrow
        h_FrontFootRef
        h_HindFootRef
    end
    
    methods
        function obj = Simulator(params, dt_sim, q_idx, q_dot_idx, decimation, ...
                                 rest_coeff, fric_coeff, ground_height, rest_coeff_joint, ...
                                 q_max_val, q_min_val, joint_idx, headless, ...
                                 CoM_fn, CoM_dot_fn, A_fn, b_fn, ...
                                 pos_front_ee, vel_front_ee, J_front_ee, ...
                                 pos_hind_ee, vel_hind_ee, J_hind_ee, keypoints_fn)
            % * Constructor to initialize the Simulator with necessary functions
            obj.p = params;
            obj.dt_sim = dt_sim;
            obj.q_idx = q_idx;
            obj.q_dot_idx = q_dot_idx;
            obj.decimation = decimation;
            obj.rest_coeff = rest_coeff;
            obj.fric_coeff = fric_coeff;
            obj.ground_height = ground_height;
            obj.rest_coeff_joint = rest_coeff_joint;
            obj.q_max_val = q_max_val;
            obj.q_min_val = q_min_val;
            obj.joint_idx = joint_idx;
            obj.CoM_fn = CoM_fn;
            obj.CoM_dot_fn = CoM_dot_fn;
            obj.A_fn = A_fn;
            obj.b_fn = b_fn;
            obj.pos_front_ee = pos_front_ee;
            obj.vel_front_ee = vel_front_ee;
            obj.J_front_ee = J_front_ee;
            obj.pos_hind_ee = pos_hind_ee;
            obj.vel_hind_ee = vel_hind_ee;
            obj.J_hind_ee = J_hind_ee;
            obj.keypoints_fn = keypoints_fn;

            obj.F_fx = 0;
            obj.F_fy = 0;
            obj.F_hx = 0;
            obj.F_hy = 0;

            % Create figure and plot handles
            obj.headless = headless;
            
            if ~headless
                obj.fig = figure;
                hold on;
    
                obj.h_OOf  = plot([0],[0],'LineWidth',2);
                obj.h_OfBf = plot([0],[0],'LineWidth',2);
                obj.h_AfCf = plot([0],[0],'LineWidth',2);
                obj.h_BfDf = plot([0],[0],'LineWidth',2);
                obj.h_CfEf = plot([0],[0],'LineWidth',2);
                obj.h_OOh  = plot([0],[0],'LineWidth',2);
                obj.h_OhBh = plot([0],[0],'LineWidth',2);
                obj.h_AhCh = plot([0],[0],'LineWidth',2);
                obj.h_BhDh = plot([0],[0],'LineWidth',2);
                obj.h_ChEh = plot([0],[0],'LineWidth',2);
                obj.ground = yline(ground_height, 'k', 'LineWidth', 1);
    
                obj.h_velocity_cmd_arrow = quiver(0, 0, 0, 0, 'g', 'MaxHeadSize', 0.5, 'LineWidth', 1.5);
    
                % Initialize arrows for front and hind leg ground reaction forces
                obj.force_scale = 20;  % Adjust scale for better visualization
                obj.h_Ff_arrow = quiver(0, 0, 0, 0, 'r', 'MaxHeadSize', 0.5, 'LineWidth', 1.5);
                obj.h_Fh_arrow = quiver(0, 0, 0, 0, 'r', 'MaxHeadSize', 0.5, 'LineWidth', 1.5);
                obj.h_FrontFootRef = plot(0, 0, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % Front foot reference marker
                obj.h_HindFootRef = plot(0, 0, 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b'); % Hind foot reference marker
                
                xlabel('x'); ylabel('y');
                obj.h_title = subtitle('t=0.0s');
    
                axis equal;
                axis([-.2 .2 -.1 .3]);
            end
        end
        
        function z_next = simulate(obj, z, u)
            % * Method to simulate the system dynamics, input u is zero-order holded.
            z_next = z;
            dz = obj.dynamics(z_next, u);
            z_next = z_next + dz * obj.dt_sim; % Compute pre-impact velocity
            for i = 1:5
                z_next(obj.q_dot_idx) = full(obj.discrete_impact_contact(z_next)); % Update velocity with contact constraints
                z_next(obj.q_dot_idx) = full(obj.joint_limit_constraint(z_next)); % Update velocity with joint limit constraint
                z_next(obj.q_idx) = z(obj.q_idx) + z_next(obj.q_dot_idx) * obj.dt_sim;
            end
        end

        function u_interp = interpolateOptimizedControl(~, t_span, u_soln, t_sim, interp_scheme)
            % * Method to interpolate the optimized control solution
            u_interp = [interp1(t_span(1:end-1), u_soln(1, :), t_sim, interp_scheme);
                        interp1(t_span(1:end-1), u_soln(2, :), t_sim, interp_scheme);
                        interp1(t_span(1:end-1), u_soln(3, :), t_sim, interp_scheme);
                        interp1(t_span(1:end-1), u_soln(4, :), t_sim, interp_scheme)];
        end

        function dz = dynamics(obj, z, u)
            A = full(obj.A_fn(z, obj.p));
            b = full(obj.b_fn(z, u, obj.p));
            qdd = A \ b;
            dz = 0*z;
            
            % Form dz
            dz(obj.q_idx) = z(obj.q_dot_idx);
            dz(obj.q_dot_idx) = qdd;
        end

        function qdot = joint_limit_constraint(obj, z)
            % Joint limits upper
            up_lim = obj.q_max_val(obj.joint_idx);
            % Joint limits lower
            low_lim = obj.q_min_val(obj.joint_idx);

            % Joint angles
            q = z(obj.q_idx);
            th = q(obj.joint_idx);
            % Joint Velocity
            q_dot = z(obj.q_dot_idx);
            dth = q_dot(obj.joint_idx);

            % * Method to handle discrete impact contact
            M = obj.A_fn(z, obj.p);
            qdot = z(obj.q_dot_idx); % Pre-impact velocity
            delta_qdot = zeros(size(qdot));
            
            % Loop through joints
            for i = 1:length(th)
                J_rot = zeros(1,length(qdot));
                J_rot(obj.joint_idx(i)) = 1;
                OSIM_rot = 1 / (J_rot / M * J_rot');
                low_C_th = th(i) - low_lim(i);
                up_C_th = th(i) - up_lim(i);
                % Lower bound
                if (dth(i) < 0) && (low_C_th < 0)
                    % Compute impulse torque
                    Torq_low = OSIM_rot * (-obj.rest_coeff_joint * dth(i) - J_rot * qdot);
                    % Compute change in  vel
                    delta_qdot = delta_qdot + M \ J_rot' * Torq_low;
                    
                % Upper bound
                elseif (dth(i) > 0) && (up_C_th > 0)
                    % Compute impulse torque
                    Torq_up = OSIM_rot * (-obj.rest_coeff_joint * dth(i) - J_rot * qdot);
                    % Compute change in  vel
                    delta_qdot = delta_qdot + M \ J_rot' * Torq_up;
                    
                end
            end
            % Update qdot
            qdot = qdot + delta_qdot;

        end
        
        function qdot = discrete_impact_contact(obj, z)
            % * Method to handle discrete impact contact
            M = obj.A_fn(z, obj.p);
            qdot = z(obj.q_dot_idx); % Pre-impact velocity

            % * Front leg contact update
            r_Ef = obj.pos_front_ee(z, obj.p);
            v_Ef = obj.vel_front_ee(z, obj.p);
            J_f  = obj.J_front_ee(z, obj.p);
            J_fx = J_f(1, :);
            J_fy = J_f(2, :);
            OSIM_fx = 1 / (J_fx / M * J_fx');
            OSIM_fy = 1 / (J_fy / M * J_fy');

            err_fy = r_Ef(2) - obj.ground_height;
            err_fdy = v_Ef(2);

            if (full(err_fy) < 0) && (full(err_fdy) < 0) % In contact
                % * Update the velocity with normal directional constraints
                F_y = OSIM_fy * (-obj.rest_coeff * err_fdy - J_fy * qdot);
                F_y_truncated = max(F_y, 0);
                qdot = qdot + M \ J_fy' * F_y_truncated;

                % * Update the velocity with tangential directional constraints
                F_x = OSIM_fx * (-J_fx * qdot);
                max_Fx = obj.fric_coeff * F_y_truncated;  % Maximum allowed F_x based on the friction cone
                F_x_truncated = min(max(F_x, -max_Fx), max_Fx);
                qdot = qdot + M \ J_fx' * F_x_truncated;

                obj.F_fx = F_x_truncated;
                obj.F_fy = F_y_truncated;
            else
                obj.F_fx = 0;
                obj.F_fy = 0;
            end

            % * Hind leg contact update
            r_Eh = obj.pos_hind_ee(z, obj.p);
            v_Eh = obj.vel_hind_ee(z, obj.p);
            J_h  = obj.J_hind_ee(z, obj.p);
            J_hx = J_h(1, :);
            J_hy = J_h(2, :);
            OSIM_hx = 1 / (J_hx / M * J_hx');
            OSIM_hy = 1 / (J_hy / M * J_hy');

            err_hy = r_Eh(2) - obj.ground_height;
            err_hdy = v_Eh(2);

            if (full(err_hy) < 0) && (full(err_hdy) < 0) % In contact
                % * Update the velocity with normal directional constraints
                F_y = OSIM_hy * (-obj.rest_coeff * err_hdy - J_hy * qdot);
                F_y_truncated = max(F_y, 0);
                qdot = qdot + M \ J_hy' * F_y_truncated;

                % * Update the velocity with tangential directional constraints
                F_x = OSIM_hx * (-J_hx * qdot);
                max_Fx = obj.fric_coeff * F_y_truncated;  % Maximum allowed F_x based on the friction cone
                F_x_truncated = min(max(F_x, -max_Fx), max_Fx);
                qdot = qdot + M \ J_hx' * F_x_truncated;

                obj.F_hx = F_x_truncated;
                obj.F_hy = F_y_truncated;
            else
                obj.F_hx = 0;
                obj.F_hy = 0;
            end
        end

        function render(obj, obs, vx_cmd)
            % Retrieve the observation
            t = obs.t;
            z = obs.z;

            % Update the plot based on the current time and state z
            keypoints = full(obj.keypoints_fn(z, obj.p));
            front_foot_ref = obs.foot_ref_position(1,:);
            hind_foot_ref = obs.foot_ref_position(2,:);

            % Extract positions for each key point
            rO  = keypoints(:,1); 
            rOf = keypoints(:,2);
            rOh = keypoints(:,3);
            rAf = keypoints(:,4); 
            rBf = keypoints(:,5);
            rCf = keypoints(:,6); 
            rDf = keypoints(:,7);
            rEf = keypoints(:,8);
            rAh = keypoints(:,9);
            rBh = keypoints(:,10);
            rCh = keypoints(:,11);
            rDh = keypoints(:,12);
            rEh = keypoints(:,13);

            % Update plot data
            set(obj.h_title, 'String', sprintf('t=%.2f', t));
            set(obj.h_OOf, 'XData', [rO(1) rOf(1)], 'YData', [rO(2) rOf(2)]);
            set(obj.h_OfBf, 'XData', [rOf(1) rBf(1)], 'YData', [rOf(2) rBf(2)]);
            set(obj.h_AfCf, 'XData', [rAf(1) rCf(1)], 'YData', [rAf(2) rCf(2)]);
            set(obj.h_BfDf, 'XData', [rBf(1) rDf(1)], 'YData', [rBf(2) rDf(2)]);
            set(obj.h_CfEf, 'XData', [rCf(1) rEf(1)], 'YData', [rCf(2) rEf(2)]);
            set(obj.h_OOh, 'XData', [rO(1) rOh(1)], 'YData', [rO(2) rOh(2)]);
            set(obj.h_OhBh, 'XData', [rOh(1) rBh(1)], 'YData', [rOh(2) rBh(2)]);
            set(obj.h_AhCh, 'XData', [rAh(1) rCh(1)], 'YData', [rAh(2) rCh(2)]);
            set(obj.h_BhDh, 'XData', [rBh(1) rDh(1)], 'YData', [rBh(2) rDh(2)]);
            set(obj.h_ChEh, 'XData', [rCh(1) rEh(1)], 'YData', [rCh(2) rEh(2)]);
            
            % Update arrow for commanded x velocity
            set(obj.h_velocity_cmd_arrow, 'XData', rO(1), 'YData', 0.2, ...
                'UData', 0.5 * full(vx_cmd), 'VData', 0);
            % Update arrow for front leg ground reaction force
            set(obj.h_Ff_arrow, 'XData', rEf(1), 'YData', rEf(2), ...
                'UData', full(obj.force_scale * obj.F_fx), 'VData', full(obj.force_scale * obj.F_fy));
            % Update arrow for hind leg ground reaction force
            set(obj.h_Fh_arrow, 'XData', rEh(1), 'YData', rEh(2), ...
                'UData', full(obj.force_scale * obj.F_hx), 'VData', full(obj.force_scale * obj.F_hy));

            % Update foot reference markers for front and hind foot positions
            set(obj.h_FrontFootRef, 'XData', front_foot_ref(1), 'YData', front_foot_ref(2));
            set(obj.h_HindFootRef, 'XData', hind_foot_ref(1), 'YData', hind_foot_ref(2));

            axis([rO(1) - 0.3, rO(1) + 0.3, -0.1, 0.3]);

        end
    end
end
