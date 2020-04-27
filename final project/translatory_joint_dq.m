function C_r_dq = translatory_joint_dq(i, j, s_i_p,s_i_q, s_j_p, q)

idx_i = body_idx(i);
phi_i = q(idx_i(3));
r_i = q(idx_i(1:2));

idx_j = body_idx(j);
phi_j = q(idx_j(3));
r_j = q(idx_j(1:2));


n = omega*(rot(phi_i)*(s_i_p - s_i_q));
n_d_phi_i =-(rot(phi_i)*(s_i_p-s_i_q));

d = r_j + rot(phi_j)*s_j_p - r_i - rot(phi_i)*s_i_p;
d_d_phi_i = - omega*rot(phi_i)*s_i_p;
d_d_phi_j = omega * rot(phi_j)*s_j_p;

C_r_dq = zeros(2,6);
C_r_dq(1,1) = -n(1);
C_r_dq(1,2) = -n(2);
C_r_dq(1,3) = n_d_phi_i'*d + n'*d_d_phi_i;
C_r_dq(2,3) = 1;
C_r_dq(1,4) = n(1);
C_r_dq(1,5) = n(2);
C_r_dq(1,6) = n'*d_d_phi_j;
C_r_dq(2,6) = -1;