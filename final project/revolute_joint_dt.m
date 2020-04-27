function C_t = revolute_joint_dt(i, j, s_i, s_j, q, q_p)

idx_i = body_idx(i);
phi_i = q(idx_i(3));
phi_p_i = q_p(idx_i(3));
r_p_i = q_p(idx_i(1:2));

idx_j = body_idx(j);
r_p_j = q_p(idx_j(1:2));
phi_j = q(idx_j(3));
phi_p_j = q_p(idx_j(3));

C_t = r_p_i + omega*rot(phi_i) * s_i * phi_p_i - r_p_j - omega*rot(phi_j) * s_j * phi_p_j;
