%% Setup for 2.740 Project
clear; clc;

% Add paths
addpath(genpath('./codegen/spine'));
addpath(genpath('./utilities'));

% Setup CasADi
if ismac
    addpath(genpath("./casadi/casadi_osx"));
elseif isunix
    addpath(genpath("./casadi/casadi_linux"));
elseif ispc
    addpath(genpath("./casadi/casadi_windows"));
end
import casadi.*

% Generate dynamics + load functions for leg
% buildSpineLegDynamics_sym()

A_fn = @A_fn;
b_fn = @b_fn;
G_fn = @G_fn;
C_fn = @C_fn;
energy_fn = @energy_fn;
InvDynamics_fn = @InvDynamics_fn;
pos_front_ee = @pos_front_ee;
vel_front_ee = @vel_front_ee;
J_front_ee = @J_front_ee;
dJ_front_ee = @dJ_front_ee;
pos_hind_ee = @pos_hind_ee;
vel_hind_ee = @vel_hind_ee;
J_hind_ee = @J_hind_ee;
dJ_hind_ee = @dJ_hind_ee;
keypoints_fn = @keypoints_fn;
CoM_fn = @CoM_fn;
CoM_dot_fn = @CoM_dot_fn;

% Parameters for leg
m0 = .75;
m1 = .0393;             m2 = .0368; 
m3 = .00783;            m4 = .0155;
% I0 = 25.1 * 10^-6;
I0 = 0.1042;
I1 = 25.1 * 10^-6;      I2 = 53.5 * 10^-6;
I3 = 9.25 * 10^-6;      I4 = 22.176 * 10^-6;
l_O_Of=.1;
l_Of_Af=.011;              l_Of_Bf=.042; 
l_Af_Cf=.096;              l_Df_Ef=.091;
l_Of_m1=0.032;             l_Bf_m2=0.0344; 
l_Af_m3=0.0622;            l_Cf_m4=0.0610;
N = 18.75;
Ir = 0.0035/N^2;
% g = 9.81;    
g = 5.81;
kappa = 1.72;
damping = 0.02;

%% Parameter vector
params   = [m0 m1 m2 m3 m4 I0 I1 I2 I3 I4 Ir N l_Of_m1 l_Bf_m2 l_Af_m3 l_Cf_m4 l_O_Of l_Of_Af l_Of_Bf l_Af_Cf l_Df_Ef g kappa damping]';
q_max_val = [ 1e6  1e6  1e6  deg2rad(76.79)+0.4652   -pi/12  deg2rad(76.79)-0.4652   -pi/12  pi/2];
q_min_val = [-1e6 -1e6 -1e6 -pi/2+0.4652 -deg2rad(145.57) -pi/2-0.4652 -deg2rad(145.57) -pi/2];
joint_idx = [4 5 6 7 8];

tau_max_val = 3;
tau_min_val = -3;

q_dim = 8;                  % Number of generalized coordinates
z_init = [0; 0.215; 0.4652; pi/4+0.4652; -pi/2; pi/4-0.4652; -pi/2; 2*-0.4652;
          0.0; 0; 0; 0; 0; 0; 0; 0;];
