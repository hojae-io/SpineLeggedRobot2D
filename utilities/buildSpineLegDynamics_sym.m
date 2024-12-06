function [] = buildSpineLegDynamics_sym()
    % Define each element of q, q_dot, q_ddot, u, F, and par individually
    syms q_0 q_1 q_2 q_3 q_4 q_5 q_6 q_7 real
    syms q_dot_0 q_dot_1 q_dot_2 q_dot_3 q_dot_4 q_dot_5 q_dot_6 q_dot_7 real
    syms q_ddot_0 q_ddot_1 q_ddot_2 q_ddot_3 q_ddot_4 q_ddot_5 q_ddot_6 q_ddot_7 real
    syms u_0 u_1 u_2 u_3 real
    syms F_0 F_1 F_2 F_3 real
    syms p_0 p_1 p_2 p_3 p_4 p_5 p_6 p_7 p_8 p_9 p_10 p_11 p_12 p_13 p_14 p_15 p_16 p_17 p_18 p_19 p_20 p_21 p22  p23 real

    % Create symbolic vectors from individual elements for convenience
    q = [q_0; q_1; q_2; q_3; q_4; q_5; q_6; q_7]; % [x, y, theta, q1, q2, q3, q4, qs], qs: spine angle
    q_dot = [q_dot_0; q_dot_1; q_dot_2; q_dot_3; q_dot_4; q_dot_5; q_dot_6; q_dot_7];
    q_ddot = [q_ddot_0; q_ddot_1; q_ddot_2; q_ddot_3; q_ddot_4; q_ddot_5; q_ddot_6; q_ddot_7];
    u = [u_0; u_1; u_2; u_3];
    F = [F_0; F_1; F_2; F_3];
    par = [p_0; p_1; p_2; p_3; p_4; p_5; p_6; p_7; p_8; p_9; p_10; p_11; p_12; p_13; p_14; p_15; p_16; p_17; p_18; p_19; p_20; p_21; p22; p23];


    par_mass = par(1:5);
    par_inertia = par(6:10);
    par_inertia_rotor = par(11);
    par_gear_ratio = par(12);
    par_com_distances = par(13:16);
    par_lengths = par(17:21);
    par_gravity = par(22);
    par_kappa = par(23);
    par_damping = par(24);
    
    F_front = [F(1);F(2)];  % 2D forces for left foot
    F_hinder = [F(3); F(4)]; % 2D forces for right foot

    F2Q_2D = @(F, r) simplify(jacobian(r, q(1:2)).' * F);    % Force contributions to generalized forces in 2D
    M2Q = @(M, w) simplify(jacobian(w, q_dot).' * M);   % Moment contributions to generalized forces
    ddt = @(r) jacobian(r, [q; q_dot]) * [q_dot; q_ddot];
    

    %% Inertial Frame Unit Vectors
    i_hat = [1; 0; 0];
    j_hat = [0; 1; 0];
    k_hat = cross(i_hat, j_hat);

    %% Body kinematics
    eh_hat = cos(q(3))*i_hat + sin(q(3))*j_hat;
    ef_hat = cos(q(3) + q(8))*i_hat + sin(q(3) + q(8))*j_hat;
    r_O = [q(1); q(2); 0];
    r_O_dot = ddt(r_O);
    w_O = q_dot(3);

    r_Of = r_O + par_lengths(1) * ef_hat;
    r_Oh = r_O - par_lengths(1) * eh_hat;

    %% Front and back body kinematics
    r_bb = r_O - 0.6*par_lengths(1)*eh_hat;
    r_bb_dot = ddt(r_bb);
    r_fb = r_O + 0.6*par_lengths(1)*ef_hat;
    r_fb_dot = ddt(r_fb);

    %% Fore-leg kinematics
    e1f_hat = sin(q(3) + q(4) + q(8))*i_hat - cos(q(3) + q(4) + q(8))*j_hat;
    e2f_hat = sin(q(3) + q(4) + q(5) + q(8))*i_hat - cos(q(3) + q(4) + q(5) + q(8))*j_hat;

    r_Af = r_Of + par_lengths(2) * e1f_hat;
    r_Bf = r_Of + par_lengths(3) * e1f_hat;
    r_Cf = r_Af + par_lengths(4) * e2f_hat;
    r_Df = r_Bf + par_lengths(4) * e2f_hat;
    r_Ef = r_Df + par_lengths(5) * e1f_hat;

    r_m1f = r_Of + par_com_distances(1) * e1f_hat;
    r_m2f = r_Bf + par_com_distances(2) * e2f_hat;
    r_m3f = r_Af + par_com_distances(3) * e2f_hat;
    r_m4f = r_Cf + par_com_distances(4) * e1f_hat;

    r_Af_dot = ddt(r_Af);
    r_Bf_dot = ddt(r_Bf);
    r_Cf_dot = ddt(r_Cf);
    r_Df_dot = ddt(r_Df);
    r_Ef_dot = ddt(r_Ef);

    r_m1f_dot = ddt(r_m1f);
    r_m2f_dot = ddt(r_m2f);
    r_m3f_dot = ddt(r_m3f);
    r_m4f_dot = ddt(r_m4f);

    w_1f = q_dot(3) + q_dot(4) + q_dot(8);
    w_2f = q_dot(3) + q_dot(4) + q_dot(5) + q_dot(8);
    w_3f = q_dot(3) + q_dot(4) + q_dot(5) + q_dot(8);
    w_4f = q_dot(3) + q_dot(4) + q_dot(8);

    %% Hind-leg kinematics
    e1h_hat = sin(q(3) + q(6))*i_hat - cos(q(3) + q(6))*j_hat;
    e2h_hat = sin(q(3) + q(6) + q(7))*i_hat - cos(q(3) + q(6) + q(7))*j_hat;

    r_Ah = r_Oh + par_lengths(2) * e1h_hat;
    r_Bh = r_Oh + par_lengths(3) * e1h_hat;
    r_Ch = r_Ah + par_lengths(4) * e2h_hat;
    r_Dh = r_Bh + par_lengths(4) * e2h_hat;
    r_Eh = r_Dh + par_lengths(5) * e1h_hat;

    r_m1h = r_Oh + par_com_distances(1) * e1h_hat;
    r_m2h = r_Bh + par_com_distances(2) * e2h_hat;
    r_m3h = r_Ah + par_com_distances(3) * e2h_hat;
    r_m4h = r_Ch + par_com_distances(4) * e1h_hat;

    r_Ah_dot = ddt(r_Ah);
    r_Bh_dot = ddt(r_Bh);
    r_Ch_dot = ddt(r_Ch);
    r_Dh_dot = ddt(r_Dh);
    r_Eh_dot = ddt(r_Eh);

    r_m1h_dot = ddt(r_m1h);
    r_m2h_dot = ddt(r_m2h);
    r_m3h_dot = ddt(r_m3h);
    r_m4h_dot = ddt(r_m4h);

    w_1h = q_dot(3) + q_dot(6);
    w_2h = q_dot(3) + q_dot(6) + q_dot(7);
    w_3h = q_dot(3) + q_dot(6) + q_dot(7);
    w_4h = q_dot(3) + q_dot(6);

    %% Lagrange Equations of Motion
    % T_0 = 0.5 * par_mass(1) * dot(r_O_dot, r_O_dot) + 0.5 * par_inertia(1) * w_O^2;
    % V_0 = par_mass(1) * par_gravity * dot(r_O, j_hat) + 0.5 * par_kappa * q(8)^2;

    T_0f = 0.5 * par_mass(1)/2 * dot(r_fb_dot, r_fb_dot) + 0.5 * par_inertia(1)/8 * (w_O+q_dot(8))^2;
    V_0f = par_mass(1)/2 * par_gravity * dot(r_fb, j_hat);

    T_0b = 0.5 * par_mass(1)/2 * dot(r_bb_dot, r_bb_dot) + 0.5 * par_inertia(1)/8 * w_O^2;
    V_0b = par_mass(1)/2 * par_gravity * dot(r_bb, j_hat);
    
    V_es = 0.5 * par_kappa * (q(8) + 2 * 0.4652)^2;
    
    T_1f = 0.5 * par_mass(2) * dot(r_m1f_dot, r_m1f_dot) + 0.5 * par_inertia(2) * w_1f^2;
    T_2f = 0.5 * par_mass(3) * dot(r_m2f_dot, r_m2f_dot) + 0.5 * par_inertia(3) * w_2f^2;
    T_3f = 0.5 * par_mass(4) * dot(r_m3f_dot, r_m3f_dot) + 0.5 * par_inertia(4) * w_3f^2;
    T_4f = 0.5 * par_mass(5) * dot(r_m4f_dot, r_m4f_dot) + 0.5 * par_inertia(5) * w_4f^2;
    T_1f_rotor = 0.5 * par_inertia_rotor * (par_gear_ratio * q_dot(4))^2;
    T_2f_rotor = 0.5 * par_inertia_rotor * (q_dot(4) + par_gear_ratio * q_dot(5))^2;

    V_g1f = par_mass(2) * par_gravity * dot(r_m1f, j_hat);
    V_g2f = par_mass(3) * par_gravity * dot(r_m2f, j_hat);
    V_g3f = par_mass(4) * par_gravity * dot(r_m3f, j_hat);
    V_g4f = par_mass(5) * par_gravity * dot(r_m4f, j_hat);

    T_1h = 0.5 * par_mass(2) * dot(r_m1h_dot, r_m1h_dot) + 0.5 * par_inertia(2) * w_1h^2;
    T_2h = 0.5 * par_mass(3) * dot(r_m2h_dot, r_m2h_dot) + 0.5 * par_inertia(3) * w_2h^2;
    T_3h = 0.5 * par_mass(4) * dot(r_m3h_dot, r_m3h_dot) + 0.5 * par_inertia(4) * w_3h^2;
    T_4h = 0.5 * par_mass(5) * dot(r_m4h_dot, r_m4h_dot) + 0.5 * par_inertia(5) * w_4h^2;
    T_1h_rotor = 0.5 * par_inertia_rotor * (par_gear_ratio * q_dot(6))^2;
    T_2h_rotor = 0.5 * par_inertia_rotor * (q_dot(6) + par_gear_ratio * q_dot(7))^2;

    V_g1h = par_mass(2) * par_gravity * dot(r_m1h, j_hat);
    V_g2h = par_mass(3) * par_gravity * dot(r_m2h, j_hat);
    V_g3h = par_mass(4) * par_gravity * dot(r_m3h, j_hat);
    V_g4h = par_mass(5) * par_gravity * dot(r_m4h, j_hat);

    T = simplify(T_0f + T_0b + T_1f + T_2f + T_3f + T_4f + T_1f_rotor + T_2f_rotor + ...
                       T_1h + T_2h + T_3h + T_4h + T_1h_rotor + T_2h_rotor);
    V = simplify(V_0f + V_0b + V_es + V_g1f + V_g2f + V_g3f + V_g4f + V_g1h + V_g2h + V_g3h + V_g4h);

    Q_tau1f = M2Q(u(1) * k_hat, w_1f * k_hat);
    Q_tau1f_reaction = M2Q(-u(1) * k_hat, (w_O + q_dot(8)) * k_hat);
    Q_tau2f = M2Q(u(2) * k_hat, w_2f * k_hat);
    Q_tau2f_reaction = M2Q(-u(2) * k_hat, w_1f * k_hat);

    Q_tau1h = M2Q(u(3) * k_hat, w_1h * k_hat);
    Q_tau1h_reaction = M2Q(-u(3) * k_hat, w_O * k_hat);
    Q_tau2h = M2Q(u(4) * k_hat, w_2h * k_hat);
    Q_tau2h_reaction = M2Q(-u(4) * k_hat, w_1h * k_hat);

    Q_spineDamp = M2Q(-par_damping * q_dot(8) * k_hat, q_dot(8) * k_hat);

    % Q_F_front = [F2Q_2D(F_front, r_Ef(1:2)); zeros(5, 1)];   % Extend to 7x1 by padding zeros for angles
    % Q_F_hind = [F2Q_2D(F_hind, r_Eh(1:2)); zeros(5, 1)]; % Extend to 7x1 by padding zeros for angles


    Q = simplify(Q_tau1f + Q_tau1f_reaction + Q_tau2f + Q_tau2f_reaction + ...
                 Q_tau1h + Q_tau1h_reaction + Q_tau2h + Q_tau2h_reaction + Q_spineDamp);

    keypoints = [r_O(1:2), r_Of(1:2), r_Oh(1:2), ...
                 r_Af(1:2), r_Bf(1:2), r_Cf(1:2), r_Df(1:2), r_Ef(1:2), ...
                 r_Ah(1:2), r_Bh(1:2), r_Ch(1:2), r_Dh(1:2), r_Eh(1:2)];

    E = simplify(T + V);
    L = simplify(T - V);
    EoM = ddt(jacobian(L, q_dot).') - jacobian(L, q).' - Q;
    % EoM = ddt(jacobian(L, q_dot).', [q; q_dot], [q_dot; q_ddot]) - jacobian(L, q).' - Q;
    InvDynamics = ddt(jacobian(L, q_dot).') - jacobian(L, q).';

    A = simplify(jacobian(EoM, q_ddot));
    b = -simplify(EoM - A * q_ddot);         % Non-inertial terms (Coriolis, gravity, etc.)
    G = simplify(jacobian(V, q)');
    C = simplify(EoM + Q - G - A*q_ddot);

    % b = -EoM_leg(q, q_dot, 0, u, par);
    Jf = jacobian(r_Ef, q); Jf = Jf(1:2, :);
    dJf = reshape(ddt(Jf(:)), size(Jf));
    Jh = jacobian(r_Eh, q); Jh = Jh(1:2, :);
    dJh = reshape(ddt(Jh(:)), size(Jh));
    z = [q; q_dot];
    CoM = (r_O * par_mass(1) + r_m1f * par_mass(2) + r_m2f * par_mass(3) + r_m3f * par_mass(4) + r_m4f * par_mass(5) + ...
                               r_m1h * par_mass(2) + r_m2h * par_mass(3) + r_m3h * par_mass(4) + r_m4h * par_mass(5)) / ...
          (par_mass(1) + par_mass(2) + par_mass(3) + par_mass(4) + par_mass(5) + ...
                         par_mass(2) + par_mass(3) + par_mass(4) + par_mass(5));
    CoM = CoM(1:2);
    CoM_dot = ddt(CoM);

    % Save each expression as a MATLAB function using matlabFunction
    % Save A as a MATLAB function
    matlabFunction(A, 'File', 'codegen/spine/A_fn', 'Vars', {z, par});
    
    % Save b as a MATLAB function
    matlabFunction(b, 'File', 'codegen/spine/b_fn', 'Vars', {z, u, par});

    % Save G as a MATLAB function
    matlabFunction(G, 'File', 'codegen/spine/G_fn', 'Vars', {z, par});
    
    % Save C as a MATLAB function
    matlabFunction(C, 'File', 'codegen/spine/C_fn', 'Vars', {z, par});
    
    % Save energy E as a MATLAB function
    matlabFunction(E, 'File', 'codegen/spine/energy_fn', 'Vars', {z, par});
    
    % Save InvDynamics as a MATLAB function
    matlabFunction(InvDynamics, 'File', 'codegen/spine/InvDynamics_fn', 'Vars', {q, q_dot, q_ddot, par});
    
    % Save the position of the front end-effector
    matlabFunction(r_Ef(1:2), 'File', 'codegen/spine/pos_front_ee', 'Vars', {z, par});
    
    % Save the velocity of the front end-effector
    matlabFunction(r_Ef_dot(1:2), 'File', 'codegen/spine/vel_front_ee', 'Vars', {z, par});
    
    % Save the Jacobian of the front end-effector
    matlabFunction(Jf, 'File', 'codegen/spine/J_front_ee', 'Vars', {z, par});
    
    % Save the derivative of the Jacobian of the front end-effector
    matlabFunction(dJf, 'File', 'codegen/spine/dJ_front_ee', 'Vars', {z, par});

    % Save the position of the hind end-effector
    matlabFunction(r_Eh(1:2), 'File', 'codegen/spine/pos_hind_ee', 'Vars', {z, par});
    
    % Save the velocity of the hind end-effector
    matlabFunction(r_Eh_dot(1:2), 'File', 'codegen/spine/vel_hind_ee', 'Vars', {z, par});
    
    % Save the Jacobian of the hind end-effector
    matlabFunction(Jh, 'File', 'codegen/spine/J_hind_ee', 'Vars', {z, par});

    % Save the derivative of the Jacobian of the hind end-effector
    matlabFunction(dJh, 'File', 'codegen/spine/dJ_hind_ee', 'Vars', {z, par});
    
    % Save the position of the front hip
    matlabFunction(r_Of(1:2), 'File', 'codegen/spine/pos_front_hip', 'Vars', {z, par});
    
    % Save the position of the hind hip
    matlabFunction(r_Oh(1:2), 'File', 'codegen/spine/pos_hind_hip', 'Vars', {z, par});
    
    % Save keypoints as a MATLAB function
    matlabFunction(keypoints, 'File', 'codegen/spine/keypoints_fn', 'Vars', {z, par});
    
    % Save the Center of Mass (CoM)
    matlabFunction(CoM, 'File', 'codegen/spine/CoM_fn', 'Vars', {z, par});
    
    % Save the derivative of the Center of Mass (CoM)
    matlabFunction(CoM_dot, 'File', 'codegen/spine/CoM_dot_fn', 'Vars', {z, par});

end