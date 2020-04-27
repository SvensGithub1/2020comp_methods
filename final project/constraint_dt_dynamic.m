function C_t = constraint_dt_dynamic(revolute, simple, translatory, driving, t, q,q_p)
r_len = length(revolute);
s_len = length(simple);
t_len = length(translatory);
d_len = length(driving);

n_constr = 2 * r_len + s_len + 2*t_len + d_len;

C_t = zeros(n_constr, 1);

c_idx = 0;
for r = revolute
    C_t(c_idx + (1:2)) = revolute_joint_dt(r.i, r.j, r.s_i, r.s_j, q, q_p);
    c_idx = c_idx + 2;
end

for s = simple
    c_idx = c_idx + 1;
    C_t(c_idx) = simple_joint_dt(s.i, s.k, q_p);
end
for trans = translatory
    C_t(c_idx + (1:2)) = translatory_joint_dt(trans.i, trans.j,trans.s_i_p,trans.s_i_q, trans.s_j_p, q, q_p);
    c_idx = c_idx + 2;
end

for d = driving
    c_idx = c_idx + 1;
    C_t(c_idx) = driving_joint_dt(d.d_k_t, t);
end

