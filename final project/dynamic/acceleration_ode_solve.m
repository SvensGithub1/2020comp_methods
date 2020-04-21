function [t q_calc q_p_calc] = acceleration_ode_solve(M,sforce,C_fun,Cq_fun,Ct_fun,g, t_0,t_end,  q_0, grav, bodies)
alpha = 10;
beta =10;
number_coordinates = 3*length(bodies);

F = @(t,q) force_vector(grav, sforce, bodies, q);
%LHS = @(t, q) [M , Cq_fun(t, q)' ; Cq_fun(t, q) , zeros(size(M,1)-size(Cq_fun(t_0,q_0),1),size(M,2)-size(Cq_fun(t_0,q_0),1))];
LHS = @(t, q) construct_LHS(M,Cq_fun(t,q),t_0, q_0,  bodies);

g_new = @(t, q,q_p) g(t, q, q_p) + 2*alpha*Ct_fun(t,q) - beta^2 *C_fun(t,q);
RHS = @(t, q, q_p) [F(t,q); g_new(t, q,q_p);q_p];
%q_ges = [q_p;lamda;q]
q_ges_0 = zeros(length(M)+length(Cq_fun(t_0,q_0)) + number_coordinates,1);
q_ges_0(end+1-number_coordinates:end) =q_0;
full_ode =@(t,q_ges) LHS(t,q_ges(end-number_coordinates+1:end))\RHS(t,q_ges(end-number_coordinates+1:end),q_ges(1:number_coordinates));
opts = odeset('AbsTol', 1e-12, 'RelTol', 1e-9);

[t, q_ges] = ode45(full_ode, [t_0 t_end], q_ges_0, opts);
q_calc =q_ges(:,end+1-number_coordinates:end);
q_p_calc = q_ges(:,1:number_coordinates);


%{
LHS = zeros(size(M)+size(Cq_fun(t_0,q_0));
LHS(1:size(M,1),1:size(M,2)) = M;
LHS(1:size(Cq_fun(t_0,q_0),1),size(M,2)+1:size(M,2)+1+size(Cq_fun(t_0,q_0),2)) = Cq_fun';
LHS(size(M,1)+1:size(M,1)+1+size(Cq_fun(t_0,q_0),1),1:size(Cq_fun(t_0,q_0),2)) = Cq_fun;
RHS
F = force_vector(grav, sforce, bodies, q);
acc = M\F;
%}