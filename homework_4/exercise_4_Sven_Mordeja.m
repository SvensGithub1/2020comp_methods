% Hands on solution for Simple Mechanism task
close all
clear
% u = [phi_2; d];
a = 0.1;
b = 0.2;
step_size=0.01;
t=0:step_size:10;
phi_1 = deg2rad(30) + t;
u_ges =zeros(2,length(t));
% set a reasonable starting point
u0 = [0; b + a];
eps = 1e-9;
J = @(u) jacobian(u, b);

for ii=1:length(t)
% create function handles
F = @(u) constraint(u, a, b, phi_1(ii));

[u, iteration_counter] = NR_method(F, J, u0, eps);
u_ges(:,ii) = u;
%fprintf('\n\tMechanism valid position is for d = %gm and phi1 = %gdeg\n\n', ...
 %   u(2), rad2deg(u(1)));
end
%[phi_2; d] = u;
figure(1)
plot(t,u_ges(1,:),'linewidth', 2)
xlabel('t [s]')
ylabel('Theta [rad]')
title('Theta over time')
saveas(figure(1),'theta_over_time','emf')

figure(2)
plot(t,u_ges(2,:),'g','linewidth', 2)
xlabel('t [s]')
ylabel('d [m]')
title('d over time')
saveas(figure(2),'d_over_time','emf')

dd_dt = diff(u_ges(2,:))/step_size;   % first derivative
dTheta_dt = diff(u_ges(1,:))/step_size;   % second derivative

figure(3)
plot(t(1:length(dd_dt)),dd_dt,'r','linewidth', 2)
xlabel('t [s]')
ylabel('v [m/s]')
title('v over time')
saveas(figure(3),'v_over_time','emf')

figure(4)
plot(t(1:length(dTheta_dt)),dTheta_dt,'k','linewidth', 2)
xlabel('t [s]')
ylabel('dTheta/dt [rad/s]')
title('angular velocity dTheta/dt over time')
saveas(figure(4),'angular_v_over_time','emf')

function P = constraint(u, a, b, phi_1)
phi_2 = u(1);
d = u(2);

P = [a * cos(phi_1) + b * cos(phi_2) - d
    a * sin(phi_1) - b * sin(phi_2)];
end

function P = jacobian(u, b)
phi_2 = u(1);
P = [-b * sin(phi_2), -1
    -b * cos(phi_2), 0];
end