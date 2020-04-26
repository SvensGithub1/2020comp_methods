function Cq = constraint_dq(revolute, simple,translatory, driving, t, q)

r_len = length(revolute);
s_len = length(simple);
t_len = length(translatory);
d_len = length(driving);

n_constr = 2 * r_len + s_len + 2*t_len + d_len;

Cq = zeros(n_constr, length(q));

c_idx = 0;
for r = revolute
    Cq(c_idx + (1:2), [body_idx(r.i), body_idx(r.j)]) = ...
        revolute_joint_dq(r.i, r.j, r.s_i, r.s_j, q);
    c_idx = c_idx + 2;
end

for s = simple
    c_idx = c_idx + 1;
    Cq(c_idx, body_idx(s.i)) = simple_joint_dq(s.k);
end

for trans = translatory
    Cq(c_idx + (1:2), [body_idx(trans.i), body_idx(trans.j)]) = ...
        translatory_joint_dq(trans.i, trans.j, trans.s_i_p, trans.s_i_q, trans.s_j_p, q);
    c_idx = c_idx + 2;
end

for d = driving
    c_idx = c_idx + 1;
    Cq(c_idx, body_idx(d.i)) = driving_joint_dq(d.k);
end