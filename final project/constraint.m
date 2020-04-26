function C = constraint(revolute, simple,translatory, driving, t, q, q_0)

r_len = length(revolute);
s_len = length(simple);
t_len = length(translatory);
d_len = length(driving);


n_constr = 2 * r_len + s_len +2*t_len + d_len;

C = zeros(n_constr, 1);

c_idx = 0;
for r = revolute
    C(c_idx + (1:2)) = revolute_joint(r.i, r.j, r.s_i, r.s_j, q);
    c_idx = c_idx + 2;
end

for s = simple
    c_idx = c_idx + 1;
    C(c_idx) = simple_joint(s.i, s.k, s.c_k, q);
end

for trans = translatory
    C(c_idx + (1:2)) = translatory_joint(trans.i, trans.j, trans.s_i_p,trans.s_i_q, trans.s_j_p, q,q_0);
    c_idx = c_idx + 2;
end

for d = driving
    c_idx = c_idx + 1;
    C(c_idx) = driving_joint(d.i, d.k, d.d_k, t, q);
end