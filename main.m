%% 2.74 Final Project
clear all; close all; clc;

%% Setup
task = 'rigid'; % 'rigid' or 'spine'
if strcmp(task, 'rigid')
    setupProject();
elseif strcmp(task, 'spine')
    setupProject_Spine();
end

%% Define parameters
q_idx = 1:q_dim;
q_dot_idx = q_dim+1:2*q_dim;
u_dim = 4;

restitution_coeff = 0.;
friction_coeff = 1.0;
ground_height = 0.;
restitution_coeff_joint = 0.;
dt_sim = 0.001;
decimation = 10;
N = 20;         % Horizon
dt_mpc = 0.01;  % Time step

headless = false;
disable_logging = false;

%% Create Logger
fps = 1/(dt_sim * decimation);
logger = Logger(fps, q_dim, q_idx, q_dot_idx, u_dim);

%% Create Environment
env = Environment(logger, z_init, params, N, dt_mpc, dt_sim, q_idx, q_dot_idx, decimation, ...
                  restitution_coeff, friction_coeff, ground_height, restitution_coeff_joint, ...
                  q_max_val, q_min_val, joint_idx, headless, disable_logging, ...
                  CoM_fn, CoM_dot_fn, A_fn, b_fn, ...
                  pos_front_ee, vel_front_ee, J_front_ee, ...
                  pos_hind_ee, vel_hind_ee, J_hind_ee, keypoints_fn);

%% Create MPC controller
mpc_controller = ConvexMPCController(N, dt_mpc, params, q_idx, q_dot_idx, ...
                                     A_fn, b_fn, G_fn, C_fn, InvDynamics_fn, pos_front_ee, vel_front_ee, J_front_ee, dJ_front_ee, ...
                                     pos_hind_ee, vel_hind_ee, J_hind_ee, dJ_hind_ee);   

%% Keyboard Control flags
if ~headless
    set(gcf, 'KeyPressFcn', @(src, event) togglePause(event));  % Set up key press listener
end
global isPaused quitSimulation vx_cmd isReset
isPaused = false;
quitSimulation = false;
vx_cmd = 0;
isReset = false;

%% Run the Experiment

obs = env.step(zeros(u_dim,1));

while ~quitSimulation
    % Check if the simulation should be paused
    if isPaused
        pause(0.01);  % Small pause to save CPU
        continue;  % Skip to the next iteration if paused
    end

    if isReset
        env.reset();
        vx_cmd = 0;
        isReset = false;
    end
    
    % Compute the optimal control solution using the MPC controller
    u = mpc_controller.compute(obs);

    % Run the environment step
    obs = env.step(u);

    % Update command
    env.vx_cmd = vx_cmd;

    % Pause to allow rendering and create a smooth animation
    pause(1e-5); % Adjust this value to control the speed of the render updates
end

% Save the data & End the simulation
logger.save();
disp('Simulation ended and video saved.');


%% [Functions]:
% Function to toggle the pause/resume state with space bar and quit with Esc
function togglePause(event)
    global isPaused quitSimulation vx_cmd isReset
    if strcmp(event.Key, 'v')
        isPaused = ~isPaused;  % Toggle pause state
    elseif strcmp(event.Key, 'escape')
        quitSimulation = true;  % Quit the simulation
    elseif strcmp(event.Key, 'l')  % Set x-velocity command to +x
        vx_cmd = vx_cmd + 0.1;
    elseif strcmp(event.Key, 'h')  % Set x-velocity command to -x
        vx_cmd = vx_cmd - 0.1;
    elseif strcmp(event.Key, 'r')  % Reset the environment
        isReset = true;
    end
end
