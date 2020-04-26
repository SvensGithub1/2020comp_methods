function g_equation = g_equation_translatory_joint(i, j,s_i_p,s_i_q, s_j_p, q, q_p)
idx_i = body_idx(i);
phi_i = q(idx_i(3));
phi_i_d_t = q_p(idx_i(3));

idx_j = body_idx(j);
phi_j = q(idx_j(3));
phi_j_d_t = q_p(idx_j(3));


n = omega*(rot(phi_i)*(s_i_p-s_i_q));
n_d_t = -phi_i_d_t*(rot(phi_i)*(s_i_p-s_i_q));
n_d_tt = -phi_i_d_t.^2*omega*(rot(phi_i)*(s_i_p-s_i_q));
d = rot(phi_j)*s_j_p - rot(phi_i)*s_i_p;
d_d_t = omega*rot(phi_j)*s_j_p*phi_j_d_t - omega*rot(phi_i)*s_i_p*phi_i_d_t;
d_d_tt = -rot(phi_j)*s_j_p*(phi_j_d_t)^2 + rot(phi_i)*s_i_p*(phi_i_d_t)^2;
g_equation = zeros(2,1);
g_equation(1,1) = n_d_tt'*d + 2*n_d_t'*d_d_t + n'*d_d_tt;