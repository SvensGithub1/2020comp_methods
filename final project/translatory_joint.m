function C_r = translatory_joint(i, j, s_i_p,s_i_q, s_j_p, q, q_0)

idx_i = body_idx(i);
phi_i = q(idx_i(3));
phi_i_0 = q_0(idx_i(3));
r_i = q(idx_i(1:2));



idx_j = body_idx(j);
phi_j = q(idx_j(3));
phi_j_0 = q_0(idx_j(3));
r_j = q(idx_j(1:2));

n = omega*(rot(phi_i)*(s_i_p - s_i_q));
d = r_j + rot(phi_j)*s_j_p - r_i - rot(phi_i)*s_i_p;

C_r(1,1) = n'*d;
C_r(2,1) = phi_i - phi_j -(phi_i_0 - phi_j_0);