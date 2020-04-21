function g_equation = g_equation_driving_joint(dd_k_t, t)
% i - body id
% k = 1, 2, 3 for x, y and phi
% d_k - function of time that define coordinate value
% q - coordinate vector

g_equation = -dd_k_t(t);