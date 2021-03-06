function g = constraint_g(revolute, simple,translatory, driving, t, q, q_p)

r_len = length(revolute);
s_len = length(simple);
t_len = length(translatory);
d_len = length(driving);

n_constr = 2 * r_len + s_len + 2*t_len + d_len;

g = zeros(n_constr, 1);

c_idx = 0;
for r = revolute
    g(c_idx + (1:2)) = g_equation_revolute_joint(r.i, r.j, r.s_i, r.s_j, q, q_p);
    c_idx = c_idx + 2;
end

for s = simple
    c_idx = c_idx + 1;
    g(c_idx) = g_equation_simple_joint(s.k);
end
for trans = translatory
    g(c_idx + (1:2)) = g_equation_translatory_joint(trans.i, trans.j, trans.s_i_p,trans.s_i_q, trans.s_j_p, q, q_p);
    c_idx = c_idx + 2;
end

for d = driving
    c_idx = c_idx + 1;
    g(c_idx) = g_equation_driving_joint(d.d_k_tt, t);
end