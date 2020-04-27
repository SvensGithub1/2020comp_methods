%% description
% this is a dynamic analysis of a single pendulum
% while applying a force
%% Define bodies
close 
clear

% body one is ground
body(1).m = 200; % mass equals to one kg is arbitray here
l = 1; 
body(1).Ic = body(1).m * l^2 / 12; % mass moment of inertia is arbitrary here
body(1).q = [0;0;0]; %starting position

body(2).m = 2; % mass equals to one kg
l = 1; 
body(2).Ic = body(2).m * l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(2).q = [0;-1;0];
%{
body(3).m = 1000; % mass equals to one kg
l = 1; 
body(3).Ic = body(3).m * l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(3).q = [0;0;0];
%}
grav = [0; -9.81]; % gravitational acceleration

%% Get mass matrix
M = mass_matrix(body);
q_0 = system_coordinates(body);


%% Add single force to the system
sforce.f = [10; 0]; % add force on pendulum 
sforce.i = 2;
sforce.u_i = [0; -1];

F = force_vector(grav, sforce, body, q_0);
%% Revolute joints
% 1 connects ground and arm
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [0; 1];

%% Simple constraints

% Three simple joints to fix the ground origin
simple(1).i = 1;
simple(1).k = 1;
simple(1).c_k = 0;

simple(2).i = 1;
simple(2).k = 2;
simple(2).c_k = 0;

simple(3).i = 1;
simple(3).k = 3;
simple(3).c_k = 0;
%% translatory constrains
translatory =[];
%% driving constrains
driving = [];

%% Solve constraint equation using NR for position and velocity
C_fun = @(t, q) constraint(revolute, simple, translatory, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, translatory,driving, t, q);
Ct_fun = @(t, q, q_p) constraint_dt(revolute, simple, translatory,driving, t, q, q_p);
g =  @(t, q, q_p) constraint_g(revolute, simple, translatory,driving, t, q, q_p);

%%[T, Q, QP, QPP] = pos_vel_acc_NR(C_fun, Cq_fun, Ct_fun,g, 6, q_0, 0.1);

%% Solvin ode
t_0 = 0;
t_end = 3;


[t, u, v , a] = acceleration_ode_solve(M,sforce,C_fun,Cq_fun,Ct_fun,g, t_0,t_end,  q_0, grav, body);
%% veryfining plots
figure
hold on
plot(u(:, 4), u(:, 5),'lineWidth', 2)
plot(u(:, 1), u(:, 2),'*','lineWidth', 2)
xlabel ('x')
ylabel ('x')
title ('position of pendulum')
legend('pendulum','origin') 
axis equal

figure
plot(t, u(:, 4),'lineWidth', 2)
hold on
plot(t, v(:, 4),'lineWidth', 2)
plot(t, a(:, 4),'lineWidth', 2)

xlabel ('t')
title ('position, velocity and acceleration of pendulum')
legend('position [m]', 'velocity [m/s]', 'acceleration [m/s^2]')
