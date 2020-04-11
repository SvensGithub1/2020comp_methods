


% solution for simple slider-crank mechanism
close all
clear
% u = [theta; d];
a = 0.1;
b = 0.2;
step_size=0.01;
t=0:step_size:10;
phi = deg2rad(30) + t;
u_ges =zeros(2,length(t));
% set a reasonable starting point
u0 = [0; b + a];
eps = 1e-9; %accuracy of computation
J = @(u) jacobian(u, b); % create function handles
for ii=1:length(t) % compute results for differnt time steps
F = @(u) constraint(u, a, b, phi(ii));% create function handles

[u, iteration_counter] = NR_method(F, J, u0, eps);
u_ges(:,ii) = u;
%fprintf('\n\tMechanism valid position is for d = %gm and phi1 = %gdeg\n\n', ...
 %   u(2), rad2deg(u(1)));
end
% u_ges(1,:) is theta calculated
% u_ges(2,:) is d calculated
%% plots
figure(1)
plot(t,u_ges(1,:),'linewidth', 2)
xlabel('t [s]')
ylabel('\theta [rad]')
title('\theta over time')

figure_1_gca=gca;


figure(2)
plot(t,u_ges(2,:),'linewidth', 2)
xlabel('t [s]')
ylabel('d [m]')
title('d over time')

figure_2_gca=gca;

dd_dt = diff(u_ges(2,:))/step_size;   % first derivative
dTheta_dt = diff(u_ges(1,:))/step_size;   % second derivative

figure(3)
plot(t(1:length(dd_dt)),dd_dt,'r','linewidth', 2)
xlabel('t [s]')
ylabel('v [m/s]')
title('v over time')

figure_3_gca=gca;

figure(4)
plot(t(1:length(dTheta_dt)),dTheta_dt,'r','linewidth', 2)
xlabel('t [s]')
ylabel('d\theta/dt [rad/s]')
title('d\theta/dt over time')
figure_4_gca=gca;

set([figure_1_gca figure_2_gca figure_3_gca figure_4_gca],'LineWidth',1,'fontsize',18,'FontName','Times New Roman')


saveas(figure(1),'theta_over_time','emf')
saveas(figure(2),'d_over_time','emf')
saveas(figure(3),'v_over_time','emf')
saveas(figure(4),'angular_v_over_time','emf')
%% constraons and jacobian matrix construction
function P = constraint(u, a, b, phi)
theta = u(1);
d = u(2);

P = [a * cos(phi) + b * cos(theta) - d
    a * sin(phi) - b * sin(theta)];
end

function P = jacobian(u, b)
theta = u(1);
P = [-b * sin(theta), -1
    -b * cos(theta), 0];
end