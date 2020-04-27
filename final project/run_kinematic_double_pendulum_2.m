%% description
% this is a kinematic analysis of a double pendulum
% while applying 2 driving constraints  
close all
clear
%% Coordinates
% ground
q1 = [0; 0; 0];
% arm 1
q2 = [0
    -0.5
    0];
% arm 2
q3 =[0
    -1.5
    0];;
q_0 = [q1; q2; q3];
%% We need two constraint types (geometric ones)
% - revolute
% - simple constraints

%% Revolute joints
% 1 connects ground and arm1
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [0; 0.5];

% 2 connects arm 1 and 2
revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [0; -0.5];
revolute(2).s_j = [0; 0.5];

% % Check revolute joint constraints
% r = revolute(3);
% C_r_i = revolute_joint(r.i, r.j, r.s_i, r.s_j, q_0)

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



% % check simple constraints
% for s = simple
%     C_s_i = simple_joint(s.i, s.k, s.c_k, q_0)
% end
%% translatory constraints
translatory= [];
%% Add some driving constraints
driving(1).i = 2;
driving(1).k = 3;
driving(1).d_k = @(t) -0.1 * t;
driving(1).d_k_t = @(t) -0.1;
driving(1).d_k_tt = @(t) 0;

driving(2).i = 3;
driving(2).k = 3;
driving(2).d_k = @(t) -0.5 * t;
driving(2).d_k_t = @(t) -0.5;
driving(2).d_k_tt = @(t) 0;

% % Verify
% d = driving(1);
% C_d_i = driving_joint(d.i, d.k, d.d_k, 0, q_0)

% %% Verify constraint function
% clc
% C = constraint(revolute, simple, driving, 0, q_0)


%% Solve constraint equation using NR for position and velocity
C_fun = @(t, q) constraint(revolute, simple, translatory, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, translatory, driving, t, q);
Ct_fun = @(t, q, q_p) constraint_dt(revolute, simple, translatory, driving, t, q, q_p);
g =  @(t, q, q_p) constraint_g(revolute, simple, translatory, driving, t, q, q_p);

[T, Q, QP, QPP] = pos_vel_acc_NR(C_fun, Cq_fun, Ct_fun,g, 100, q_0, 0.1);


%% Some verification plots
plot(Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
xlabel ('x [m]')
ylabel ('y [m]')
title ('positions of pendulums')
legend('pendulum 1', 'pendulum 2', 'origin')
handle=gca;
set(handle,'LineWidth',1,'fontsize',18,'FontName','Times New Roman')
saveas(figure(1),'kin_double_pen_pos','emf')
figure
%% Some verification plots
plot(QP(:, 4), QP(:, 5), ...
    QP(:, 7), QP(:, 8), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
xlabel ('v_x [m/s]')
ylabel ('v_y [m/s]')
title ('velocity of bodies')
legend('pendulum 1', 'pendulum 2', 'origin')

%% veryfy acceleration using numerical diff
%{ 

figure
plot(diff(QP(:, 4))./0.1, diff(QP(:, 5))./0.1, ...
    diff(QP(:, 7))./0.1, diff(QP(:, 8))./0.1, ...
    diff(QP(:, 10))./0.1, diff(QP(:, 11))./0.1, ...
    0, 0, '*', 'LineWidth', 2);
axis equal
%}

figure
plot(QPP(:, 4), QPP(:, 5), ...
    QPP(:, 7), QPP(:, 8), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
xlabel ('a_x [m/s^2]')
ylabel ('a_y [m/s^2]')
title ('acceleration of bodies')
legend('body 1', 'body 2', 'origin')

%% plot over time
figure
hold on
plot(T, Q(:, 7),'LineWidth', 2);
plot(T, QP(:, 7),'LineWidth', 2);
plot(T, QPP(:, 7),'LineWidth', 2);
xlabel ('t [s]')
title ({'position, velocity and acceleration in x-direction'; 'of pendulum 2'})
legend('position [m]', 'velocity [m/s]', 'acceleration [m/s^2]')
handle=gca;
set(handle,'LineWidth',1,'fontsize',18,'FontName','Times New Roman')
saveas(figure(4),'kin_double_pen_ov_time','emf')    