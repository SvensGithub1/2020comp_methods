function C_t = simple_joint_dt(i, k, q_p)
% i - body id
% k = 1, 2, 3 for x, y and phi
% c_k - value of coordinate to keep all the time
% q - coordinate vector

idx_i = body_idx(i);
%C_t = 0;
C_t = q_p(idx_i(k));