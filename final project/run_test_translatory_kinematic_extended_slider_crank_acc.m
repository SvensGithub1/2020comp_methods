%% description
% this is a kinematic analysis of a Slider crank mechanism
% in stead of using a simple constraint a translatory is used
% the results are the same as run_kinematic_two_pendulums
clear
%% Coordinates
% ground 
q1 = [0; 0; 0];
% crank
q2 = [-0.1 * cosd(30)
    0.1 * sind(30)
    -deg2rad(30)];
% link
h_B = 0.2 * sind(30); % y coordinate of point B
phi_l = asin(h_B / 0.5); % link's angle
q3 = [-0.2 * cosd(30) - 0.3 * cos(phi_l)
    h_B - 0.3 * sin(phi_l)
    phi_l];
% slider
q4 = [-0.2 * cosd(30) - 0.5 * cos(phi_l)
    0
    0];

q_0 = [q1; q2; q3; q4]; % initial coordinates

%% We need two constraint types (geometric ones)
% - revolute
% - simple constraints

%% Revolute joints
% 1 connects ground and crank
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [0.1; 0];

% 2 connects crank and link
revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [-0.1; 0];
revolute(2).s_j = [0.3; 0];

% 3 connects link and slider
revolute(3).i = 3;
revolute(3).j = 4;
revolute(3).s_i = [-0.2; 0];
revolute(3).s_j = [0; 0];

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
translatory(1).i =1;
translatory(1).j =4;
translatory(1).s_i_p = [-1;0];
translatory(1).s_i_q = [-0.5;0];
translatory(1).s_j_p = [-2; 0];
% slider - use simple joints instead of translational

%% Add some driving constraints
driving.i = 2;
driving.k = 3;
driving.d_k = @(t) -pi/6 - 1.2 * t;
driving.d_k_t = @(t) -1.2;
driving.d_k_tt = @(t) 0;

% % Verify
% d = driving(1);
% C_d_i = driving_joint(d.i, d.k, d.d_k, 0, q_0)

% %% Verify constraint function
% clc
% C = constraint(revolute, simple, driving, 0, q_0)



%% Solve constraint equation using NR for position and velocity
C_fun = @(t, q) constraint(revolute, simple, translatory, driving, t, q, q_0);
Cq_fun = @(t, q) constraint_dq(revolute, simple,translatory, driving, t, q);
Ct_fun = @(t, q, q_p) constraint_dt(revolute, simple,translatory, driving, t, q, q_p);
g =  @(t, q, q_p) constraint_g(revolute, simple,translatory, driving, t, q, q_p);

[T, Q, QP, QPP] = pos_vel_acc_NR(C_fun, Cq_fun, Ct_fun,g, 6, q_0, 0.1);



%% Some verification plots
plot(Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    Q(:, 10), Q(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
xlabel ('x')
ylabel ('y')
title ('positions of bodies')
legend('body 1', 'body 2', 'body 3', 'origin')
figure
%% Some verification plots
plot(QP(:, 4), QP(:, 5), ...
    QP(:, 7), QP(:, 8), ...
    QP(:, 10), QP(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
xlabel ('v_x')
ylabel ('v_y')
title ('velocity of bodies')
legend('body 1', 'body 2', 'body 3', 'origin')

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
    QPP(:, 10), QPP(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
xlabel ('a_x')
ylabel ('a_y')
title ('acceleration of bodies')
legend('body 1', 'body 2', 'body 3', 'origin')

%% plot over time
figure
hold on
plot(T, Q(:, 10),'LineWidth', 2);
plot(T, QP(:, 10),'LineWidth', 2);
plot(T, QPP(:, 10),'LineWidth', 2);
xlabel ('t')
title ('position, velocity and acceleration of body 3')
legend('position [m]', 'velocity [m/s]', 'acceleration [m/s^2]')
    