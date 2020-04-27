function [t, q_calc, q_p_calc,q_pp_calc] = acceleration_ode_solve(M,sforce,C_fun,Cq_fun,Ct_fun,g, t_0,t_end,  q_0, grav, bodies)
alpha = 10;
beta =10;
number_coordinates = 3*length(bodies);

F = @(t,q) force_vector(grav, sforce, bodies, q); % construct force vectors
LHS = @(t, q) construct_LHS(M,Cq_fun(t,q),  bodies); % construct left hand side
g_new = @(t, q,q_p) g(t, q, q_p) + 2*alpha*Ct_fun(t,q,q_p) - beta^2 *C_fun(t,q); % construct new g vector
RHS = @(t, q, q_p) [F(t,q); g_new(t, q,q_p);q_p]; % construct right hand side

q_ges_0 = zeros(length(M)+length(Cq_fun(t_0,q_0)) + number_coordinates,1); % construct initiali values
q_ges_0(end+1-number_coordinates:end) =q_0;

full_ode =@(t,q_ges) LHS(t,q_ges(end-number_coordinates+1:end))\RHS(t,q_ges(end-number_coordinates+1:end),q_ges(1:number_coordinates)); %define the ode

%opts = odeset('AbsTol', 1e-6, 'RelTol', 1e-3); %specification od ode45
opts = odeset('AbsTol', 1e-15, 'RelTol', 1e-12); %specification od ode45

[t, q_ges] = ode45(full_ode, [t_0 t_end], q_ges_0, opts); %solve
q_calc =q_ges(:,end+1-number_coordinates:end);
q_p_calc = q_ges(:,1:number_coordinates);

%% calculating q_pp
q_p_ges=zeros(size(q_ges));
for ii =1:length(t)
q_p_ges(ii,:) = full_ode(t(ii),q_ges(ii,:)');
end
q_pp_calc = q_p_ges(:,1:number_coordinates);