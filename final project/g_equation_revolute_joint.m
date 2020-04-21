function g_equation = g_equation_revolute_joint(i, j, s_i, s_j, q, q_p)

idx_i = body_idx(i);
phi_i = q(idx_i(3));
phi_p_i = q_p(idx_i(3));
idx_j = body_idx(j);
phi_j = q(idx_j(3));
phi_p_j = q_p(idx_j(3));
g_equation = rot(phi_i) * s_i * phi_p_i^2- rot(phi_j) * s_j * phi_p_j^2;
