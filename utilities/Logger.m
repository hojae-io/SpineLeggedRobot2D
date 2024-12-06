classdef Logger < handle
    properties
        video_writer   % Handle to the VideoWriter object
        folder_name    % Name of the folder to store the videos
        fps
        
        % Logger array
        q_idx;
        q_dot_idx;
        log_t;
        log_q;
        log_q_dot;
        log_u;
        log_vx_cmd;
        log_front_ee;
        log_hind_ee;
        
    end
    methods
        function obj = Logger(fps, q_dim, q_idx, q_dot_idx, u_dim)
            % Setup a folder to save the data
            obj.folder_name = 'analysis';

            if ~isfolder(obj.folder_name)
                mkdir(obj.folder_name); % Create folder if it doesn't exist
            end

            % Setup the video writer with the given parameters
            obj.fps = fps;
            obj.video_writer = VideoWriter(fullfile(obj.folder_name, sprintf('%s_video.mp4', 'simulation')), 'MPEG-4');
            obj.video_writer.FrameRate = fps;
            open(obj.video_writer);

            % Initialize logger array
            obj.q_idx = q_idx;
            obj.q_dot_idx = q_dot_idx;
            
            obj.log_t = 0;
            obj.log_q = zeros(q_dim, 1);
            obj.log_q_dot = zeros(q_dim, 1);
            obj.log_u = zeros(u_dim, 1);
            obj.log_vx_cmd = 0;
            obj.log_front_ee = zeros(2,1);
            obj.log_hind_ee = zeros(2,1);

            fprintf('Logger initialized with folder: %s\n', obj.folder_name);
        end

        function log(obj, frame, obs, u, vx_cmd, keypoints)
            % Log data
            obj.write_video(frame);
            obj.log_t = [obj.log_t, obs.t];
            obj.log_q = [obj.log_q, obs.z(obj.q_idx)];
            obj.log_q_dot = [obj.log_q_dot, obs.z(obj.q_dot_idx)];
            obj.log_u = [obj.log_u, u];
            obj.log_vx_cmd = [obj.log_vx_cmd, vx_cmd];
            
            front_ee = keypoints(:,8) - keypoints(:,2);
            hind_ee = keypoints(:,13) - keypoints(:,3);
            obj.log_front_ee = [obj.log_front_ee, front_ee];
            obj.log_hind_ee = [obj.log_hind_ee, hind_ee];
        end

        function save(obj)
            % Save data
            obj.close_video();

            obj.plot_body_position_trajectory();
            obj.plot_joint_position_trajectory();
            obj.plot_body_velocity_trajectory();
            obj.plot_joint_velocity_trajectory();
            obj.plot_control_input_trajectory();
            obj.plot_velocity_tracking_performance();
            obj.plot_foot_position_trajectory();

            % obj.animate_joint_position_trajectory();
            % obj.animate_velocity_tracking_performance();

            obj.save_joint_position_trajectory();
            obj.save_joint_velocity_trajectory();
            obj.save_foot_position_trajectory();
        end

        function write_video(obj, frame)
            writeVideo(obj.video_writer, frame);
        end

        function close_video(obj)
            % Close the video writer if it was opened
            if ~isempty(obj.video_writer)
                close(obj.video_writer);
            end
        end

        function plot_body_position_trajectory(obj)
            % Plot body position and orientation trajectory
            fig_body = figure('Visible', 'off');
            hold on;

            % Set left y-axis for position (meters)
            yyaxis left;
            plot(obj.log_t, obj.log_q(1,:), 'r-', 'DisplayName', 'x (Position)');
            plot(obj.log_t, obj.log_q(2,:), 'g-', 'DisplayName', 'y (Position)');
            ylabel('Position (m)'); % Label for left y-axis

            % Set left axis color to black
            ax = gca; % Get current axes
            ax.YColor = 'k'; % Set left y-axis color to black

            % Set right y-axis for angle (radians)
            yyaxis right;
            plot(obj.log_t, obj.log_q(3,:), 'b-', 'DisplayName', '\theta (Orientation)');
            ylabel('Angle (rad)'); % Label for right y-axis

            % Set right axis color to black
            ax.YColor = 'k'; % Set right y-axis color to black

            % Set common x-axis label and title
            xlabel('Time (s)');
            title('Body Position and Orientation Trajectory');

            % Adjust x-axis limits to remove extra space
            xlim([min(obj.log_t), max(obj.log_t)]);

            % Add a legend
            legend('Location', 'northeast');

            % Save the plot
            saveas(fig_body, fullfile(obj.folder_name, 'Body_Position_Trajectory.pdf'));

            % Close the figure
            close(fig_body);
        end

        function plot_joint_position_trajectory(obj)
            % Plot joint position trajectory
            fig_joint = figure('Visible', 'off');
            hold on;

            % Plot joint position dimensions
            plot(obj.log_t, obj.log_q(4,:), 'm-', 'DisplayName', '\theta_{1f} (Front Hip)');
            plot(obj.log_t, obj.log_q(5,:), 'c-', 'DisplayName', '\theta_{2f} (Front Knee)');
            plot(obj.log_t, obj.log_q(6,:), 'y-', 'DisplayName', '\theta_{1h} (Hind Hip)');
            plot(obj.log_t, obj.log_q(7,:), 'k-', 'DisplayName', '\theta_{2h} (Hind Knee)');

            % Set labels, title, and legend
            xlabel('Time (s)');
            ylabel('Joint Angle (rad)');
            title('Joint Position Trajectory');
            legend('Location', 'northeast');

            % Adjust x-axis limits to remove extra space
            xlim([min(obj.log_t), max(obj.log_t)]);

            % Save the plot
            saveas(fig_joint, fullfile(obj.folder_name, 'Joint_Position_Trajectory.pdf'));

            % Close the figure
            close(fig_joint);
        end

        function animate_joint_position_trajectory(obj)
            % Create a new figure for animation with visibility off
            fig_joint = figure('Visible', 'off');
            hold on;
        
            % Initialize the animated plots
            h1 = plot(NaN, NaN, 'm-', 'DisplayName', '\theta_{1f} (Front Hip)', 'LineWidth', 2);
            h2 = plot(NaN, NaN, 'c-', 'DisplayName', '\theta_{2f} (Front Knee)', 'LineWidth', 2);
            h3 = plot(NaN, NaN, 'g-', 'DisplayName', '\theta_{1h} (Hind Hip)', 'LineWidth', 2);
            h4 = plot(NaN, NaN, 'k-', 'DisplayName', '\theta_{2h} (Hind Knee)', 'LineWidth', 2);
        
            % Set labels, title, and legend
            xlabel('Time (s)');
            ylabel('Joint Angle (rad)');
            title('Joint Position Trajectory');
            legend('Location', 'northeast');
        
            % Adjust x-axis limits to remove extra space
            xlim([min(obj.log_t), max(obj.log_t)]);
            ylim([min(min(obj.log_q(4:7, :))) - 0.5, max(max(obj.log_q(4:7, :))) + 0.5]);
        
            % Prepare video writer
            video_filename = fullfile(obj.folder_name, 'Joint_Position_Trajectory.mp4');
            v = VideoWriter(video_filename, 'MPEG-4');
            v.FrameRate = obj.fps; % Adjust frame rate
            open(v);
        
            % Animation loop
            num_points = length(obj.log_t);
            for i = 1:num_points
                % Update the plot data for each joint angle
                set(h1, 'XData', obj.log_t(1:i), 'YData', obj.log_q(4, 1:i));
                set(h2, 'XData', obj.log_t(1:i), 'YData', obj.log_q(5, 1:i));
                set(h3, 'XData', obj.log_t(1:i), 'YData', obj.log_q(6, 1:i));
                set(h4, 'XData', obj.log_t(1:i), 'YData', obj.log_q(7, 1:i));
        
                % Capture the current frame
                frame = getframe(fig_joint);
                writeVideo(v, frame);
            end
        
            % Close the video writer
            close(v);
        
            % Close the figure
            close(fig_joint);
        
            disp(['Animation saved as ', video_filename]);
        end

        function plot_body_velocity_trajectory(obj)
            % Plot body velocity trajectory
            fig_body_vel = figure('Visible', 'off');
            hold on;

            % Set left y-axis for velocity (m/s)
            yyaxis left;
            plot(obj.log_t, obj.log_q_dot(1,:), 'r-', 'DisplayName', 'v_x (Velocity)');
            plot(obj.log_t, obj.log_q_dot(2,:), 'g-', 'DisplayName', 'v_y (Velocity)');
            ylabel('Velocity (m/s)'); % Label for left y-axis

            % Set left axis color to black
            ax = gca; % Get current axes
            ax.YColor = 'k'; % Set left y-axis color to black

            % Set right y-axis for angular velocity (rad/s)
            yyaxis right;
            plot(obj.log_t, obj.log_q_dot(3,:), 'b-', 'DisplayName', '\omega (Angular Velocity)');
            ylabel('Angular Velocity (rad/s)'); % Label for right y-axis

            % Set right axis color to black
            ax.YColor = 'k'; % Set right y-axis color to black

            % Set common x-axis label and title
            xlabel('Time (s)');
            title('Body Velocity Trajectory');

            % Adjust x-axis limits to remove extra space
            xlim([min(obj.log_t), max(obj.log_t)]);

            % Add a legend
            legend('Location', 'northeast');

            % Save the plot
            saveas(fig_body_vel, fullfile(obj.folder_name, 'Body_Velocity_Trajectory.pdf'));

            % Close the figure
            close(fig_body_vel);
        end

        function plot_joint_velocity_trajectory(obj)
            % Plot joint velocity trajectory
            fig_joint_vel = figure('Visible', 'off');
            hold on;

            % Plot joint velocity dimensions
            plot(obj.log_t, obj.log_q_dot(4,:), 'm-', 'DisplayName', 'v_{1f} (Front Hip)');
            plot(obj.log_t, obj.log_q_dot(5,:), 'c-', 'DisplayName', 'v_{2f} (Front Knee)');
            plot(obj.log_t, obj.log_q_dot(6,:), 'y-', 'DisplayName', 'v_{1h} (Hind Hip)');
            plot(obj.log_t, obj.log_q_dot(7,:), 'k-', 'DisplayName', 'v_{2h} (Hind Knee)');

            % Set labels, title, and legend
            xlabel('Time (s)');
            ylabel('Joint Velocity (rad/s)');
            title('Joint Velocity Trajectory');
            legend('Location', 'northeast');

            % Adjust x-axis limits to remove extra space
            xlim([min(obj.log_t), max(obj.log_t)]);

            % Save the plot
            saveas(fig_joint_vel, fullfile(obj.folder_name, 'Joint_Velocity_Trajectory.pdf'));

            % Close the figure
            close(fig_joint_vel);
        end

        function plot_control_input_trajectory(obj)
            % Plot control input trajectory
            fig_control = figure('Visible', 'off');
            hold on;

            % Plot control input dimensions
            plot(obj.log_t, obj.log_u(1,:), 'r-', 'DisplayName', '\tau_{1f} (Front Hip)');
            plot(obj.log_t, obj.log_u(2,:), 'g-', 'DisplayName', '\tau_{2f} (Front Knee)');
            plot(obj.log_t, obj.log_u(3,:), 'b-', 'DisplayName', '\tau_{1h} (Hind Hip)');
            plot(obj.log_t, obj.log_u(4,:), 'k-', 'DisplayName', '\tau_{2h} (Hind Knee)');

            % Set labels, title, and legend
            xlabel('Time (s)');
            ylabel('Control Input (Nm)');
            title('Control Input Trajectory');
            legend('Location', 'northeast');

            % Adjust x-axis limits to remove extra space
            xlim([min(obj.log_t), max(obj.log_t)]);

            % Save the plot
            saveas(fig_control, fullfile(obj.folder_name, 'Control_Input_Trajectory.pdf'));

            % Close the figure
            close(fig_control);
        end

        function plot_velocity_tracking_performance(obj)
            % Plot velocity tracking performance
            fig_vel_track = figure('Visible', 'off');
            hold on;

            % Plot commanded and actual velocity
            plot(obj.log_t, obj.log_vx_cmd, 'k--', 'DisplayName', 'v_x (Commanded)', 'LineWidth', 2);
            plot(obj.log_t, obj.log_q_dot(1,:), 'b-', 'DisplayName', 'v_x (Measured)', 'LineWidth', 2);

            % Set labels, title, and legend
            xlabel('Time (s)');
            ylabel('Velocity (m/s)');
            title('Velocity Tracking Performance');
            legend('Location', 'northeast');

            % Adjust x-axis limits to remove extra space
            xlim([min(obj.log_t), max(obj.log_t)]);

            % Save the plot
            saveas(fig_vel_track, fullfile(obj.folder_name, 'Velocity_Tracking_Performance.pdf'));

            % Close the figure
            close(fig_vel_track);
        end

        function animate_velocity_tracking_performance(obj)
            % Create a new figure for animation
            fig_vel_track = figure('Visible', 'off');
            hold on;
            
            % Initialize the animated plots
            h_cmd = plot(NaN, NaN, 'k--', 'DisplayName', 'v_x (Commanded)', 'LineWidth', 2);
            h_meas = plot(NaN, NaN, 'b-', 'DisplayName', 'v_x (Measured)', 'LineWidth', 2);
        
            % Set labels, title, and legend
            xlabel('Time (s)');
            ylabel('Velocity (m/s)');
            title('Velocity Tracking Performance');
            legend('Location', 'northeast');
        
            % Adjust x-axis limits to remove extra space
            xlim([min(obj.log_t), max(obj.log_t)]);
            ylim([min([obj.log_vx_cmd, obj.log_q_dot(1, :)]) - 0.1, max([obj.log_vx_cmd, obj.log_q_dot(1, :)]) + 0.1]);
            
            % Prepare video writer
            video_filename = fullfile(obj.folder_name, 'Velocity_Tracking_Performance.mp4');
            v = VideoWriter(video_filename, 'MPEG-4');
            v.FrameRate = obj.fps; % Adjust frame rate
            open(v);
        
            % Animation loop
            num_points = length(obj.log_t);
            for i = 1:num_points
                % Update the commanded velocity
                set(h_cmd, 'XData', obj.log_t(1:i), 'YData', obj.log_vx_cmd(1:i));
                
                % Update the measured velocity
                set(h_meas, 'XData', obj.log_t(1:i), 'YData', obj.log_q_dot(1, 1:i));
                
                % Capture the current frame
                frame = getframe(fig_vel_track);
                writeVideo(v, frame);
        
                % Exit the loop if the figure is closed
                if ~isvalid(fig_vel_track)
                    break;
                end
            end
        
            % Close the video writer
            close(v);
            
            % Close the figure
            close(fig_vel_track);
        
            disp(['Animation saved as ', video_filename]);
        end

        function plot_foot_position_trajectory(obj)
            % Plot foot position trajectory in time domain
            fig_foot = figure('Visible', 'off');
            hold on;

            % Plot foot position dimensions
            plot(obj.log_t, obj.log_front_ee(1,:), 'r-', 'DisplayName', 'Front Foot (x)');
            plot(obj.log_t, obj.log_front_ee(2,:), 'g-', 'DisplayName', 'Front Foot (y)');
            plot(obj.log_t, obj.log_hind_ee(1,:), 'b-', 'DisplayName', 'Hind Foot (x)');
            plot(obj.log_t, obj.log_hind_ee(2,:), 'k-', 'DisplayName', 'Hind Foot (y)');

            % Set labels, title, and legend
            xlabel('Time (s)');
            ylabel('Foot Position (m)');
            title('Foot Position Trajectory');
            legend('Location', 'northeast');

            % Adjust x-axis limits to remove extra space
            xlim([min(obj.log_t), max(obj.log_t)]);

            % Save the plot
            saveas(fig_foot, fullfile(obj.folder_name, 'Foot_Position_Trajectory.pdf'));

            % Close the figure
            close(fig_foot);

            % Plot foot position trajectory in xy domain
            fig_foot_xy = figure('Visible', 'off');
            hold on;

            % Plot foot position dimensions
            plot(obj.log_front_ee(1,:), obj.log_front_ee(2,:), 'r-', 'DisplayName', 'Front Foot');
            plot(obj.log_hind_ee(1,:), obj.log_hind_ee(2,:), 'b-', 'DisplayName', 'Hind Foot');

            % Set labels, title, and legend
            xlabel('Foot X Position (m)');
            ylabel('Foot Y Position (m)');
            title('Foot Position Trajectory (XY Plane)');
            legend('Location', 'northeast');

            % Save the plot
            saveas(fig_foot_xy, fullfile(obj.folder_name, 'Foot_Position_Trajectory_XY.pdf'));

            % Close the figure
            close(fig_foot_xy);
        end

        function save_joint_position_trajectory(obj)
            % Save joint position trajectory to a CSV file
            writematrix(obj.log_q(4:end,:), fullfile(obj.folder_name, 'Joint_Position_Trajectory.csv'));
        end

        function save_joint_velocity_trajectory(obj)
            % Save joint velocity trajectory to a CSV file
            writematrix(obj.log_q_dot(4:end,:), fullfile(obj.folder_name, 'Joint_Velocity_Trajectory.csv'));
        end

        function save_foot_position_trajectory(obj)
            % Save foot position trajectory to a CSV file
            log_foot = [obj.log_front_ee; obj.log_hind_ee];
            writematrix(log_foot, fullfile(obj.folder_name, 'Foot_Position_Trajectory.csv'));
        end
    end
end